#ifndef MBEDTLS_HANDLER_HPP
#define MBEDTLS_HANDLER_HPP

#include <Arduino.h>
#include <Ethernet.h>
#include "esp_task_wdt.h"

#include "mbedtls/platform.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/base64.h"

#define TLS_HANDSHAKE_TIMEOUT 8000U
#define TLS_READ_TIMEOUT 4000U
#define TLS_WRITE_TIMEOUT 8000U
#define TLS_RECV_WAIT_MS 2U

#define TCP_CONNECT_RETRIES 1

#define SSL_BUFFER_SIZE 1024U
#define SSL_SEND_CHUNK_SIZE 1024U
#define TCP_SEND_FLUSH_DELAY_MS 1U

#define BASE64_AUTH_SIZE 192U

static const int PREFERRED_CIPHERS[] = {
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256,
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256,
    MBEDTLS_TLS_RSA_WITH_AES_128_GCM_SHA256,
    0};

#define MBEDTLS_DEBUG_ENABLE

#ifdef MBEDTLS_DEBUG_ENABLE
#define TLS_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define TLS_LOG(fmt, ...) \
    do                    \
    {                     \
    } while (0)
#endif

static int eth_ssl_send(void *ctx, const unsigned char *buf, size_t len)
{
    EthernetClient *client = static_cast<EthernetClient *>(ctx);
    if (!client || !client->connected())
        return MBEDTLS_ERR_NET_CONN_RESET;

    if (len > SSL_SEND_CHUNK_SIZE)
        len = SSL_SEND_CHUNK_SIZE;

    int written = client->write(buf, len);
    if (written > 0)
    {
        client->flush();
        return written;
    }

    return MBEDTLS_ERR_SSL_WANT_WRITE;
}

static int eth_ssl_recv(void *ctx, unsigned char *buf, size_t len)
{
    EthernetClient *client = static_cast<EthernetClient *>(ctx);
    if (!client)
        return MBEDTLS_ERR_NET_CONN_RESET;

    if (client->available() > 0)
    {
        int avail = client->available();
        int readLen = (avail > (int)len) ? (int)len : avail;
        int n = client->read(buf, readLen);
        return (n > 0) ? n : MBEDTLS_ERR_SSL_WANT_READ;
    }

    if (!client->connected())
        return MBEDTLS_ERR_NET_CONN_RESET;

    vTaskDelay(pdMS_TO_TICKS(TLS_RECV_WAIT_MS));
    return MBEDTLS_ERR_SSL_WANT_READ;
}

static bool parseJsonResponse(const char *body, size_t bodyLen)
{

    const char *p = body;
    const char *end = body + bodyLen;

    while (p < end - 14)
    {

        const char *found = strstr(p, "\"isSuccess\"");
        if (!found || found >= end)
            break;

        const char *colon = strchr(found + 11, ':');
        if (!colon || colon >= end)
            break;

        colon++;
        while (colon < end && (*colon == ' ' || *colon == '\t'))
            colon++;

        if (colon + 4 <= end && strncmp(colon, "true", 4) == 0)
            return true;

        break;
    }
    return false;
}

static String buildHttpRequest(const char *host,
                               const char *path,
                               const char *data,
                               const char *username,
                               const char *password)
{

    char authRaw[128];
    int authRawLen = snprintf(authRaw, sizeof(authRaw), "%s:%s", username, password);
    if (authRawLen < 0 || authRawLen >= (int)sizeof(authRaw))
        return "";

    unsigned char base64Auth[BASE64_AUTH_SIZE] = {};
    size_t authLen = 0;
    mbedtls_base64_encode(base64Auth, sizeof(base64Auth), &authLen,
                          (const unsigned char *)authRaw, (size_t)authRawLen);
    base64Auth[authLen] = '\0';

    size_t dataLen = strlen(data);
    String request;
    request.reserve(256 + dataLen);

    request = "POST ";
    request += path;
    request += " HTTP/1.0\r\nHost: ";
    request += host;
    request += "\r\nAuthorization: Basic ";
    request += reinterpret_cast<const char *>(base64Auth);
    request += "\r\nContent-Type: application/json\r\nContent-Length: ";
    request += String(dataLen);
    request += "\r\n\r\n";
    request += data;

    return request;
}

static bool sendRequest(mbedtls_ssl_context *ssl,
                        const String &request,
                        unsigned long timeoutMs)
{
    const char *reqBuf = request.c_str();
    size_t reqLen = request.length();
    size_t written = 0;
    unsigned long send_t = millis();

    TLS_LOG("[HTTPS] Sending %u bytes...", (unsigned)reqLen);

    while (written < reqLen)
    {
        size_t chunkSize = reqLen - written;
        if (chunkSize > SSL_SEND_CHUNK_SIZE)
            chunkSize = SSL_SEND_CHUNK_SIZE;

        int ret = mbedtls_ssl_write(
            ssl,
            reinterpret_cast<const unsigned char *>(reqBuf + written),
            chunkSize);

        if (ret > 0)
        {
            written += (size_t)ret;

            if (written < reqLen)
                vTaskDelay(pdMS_TO_TICKS(TCP_SEND_FLUSH_DELAY_MS));
        }
        else if (ret == MBEDTLS_ERR_SSL_WANT_READ ||
                 ret == MBEDTLS_ERR_SSL_WANT_WRITE)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        else
        {
            TLS_LOG(" FAIL (-0x%04X)\n", (unsigned)(-ret));
            return false;
        }

        if (millis() - send_t > timeoutMs)
        {
            TLS_LOG(" TIMEOUT\n");
            return false;
        }

        esp_task_wdt_reset();
    }

    TLS_LOG(" OK (%u bytes)\n", (unsigned)written);
    return true;
}

int perform_https_request_mbedtls(EthernetClient &ethClient,
                                  const char *host,
                                  const char *path,
                                  const char *data,
                                  const char *username,
                                  const char *password)
{
    int ret = -1;

    mbedtls_entropy_context *entropy = new mbedtls_entropy_context();
    mbedtls_ctr_drbg_context *ctr_drbg = new mbedtls_ctr_drbg_context();
    mbedtls_ssl_context *ssl = new mbedtls_ssl_context();
    mbedtls_ssl_config *conf = new mbedtls_ssl_config();
    mbedtls_x509_crt *cacert = new mbedtls_x509_crt();

    if (!entropy || !ctr_drbg || !ssl || !conf || !cacert)
    {
        TLS_LOG("[TLS] HEAP FAIL: free=%u\n", (unsigned)ESP.getFreeHeap());
        delete entropy;
        delete ctr_drbg;
        delete ssl;
        delete conf;
        delete cacert;
        return -1;
    }

    mbedtls_ssl_init(ssl);
    mbedtls_ssl_config_init(conf);
    mbedtls_x509_crt_init(cacert);
    mbedtls_ctr_drbg_init(ctr_drbg);
    mbedtls_entropy_init(entropy);

    TLS_LOG("\n[HTTPS] -> %s%s\n", host, path);
    TLS_LOG("[HEAP]  free=%u  min=%u\n",
            (unsigned)ESP.getFreeHeap(),
            (unsigned)ESP.getMinFreeHeap());

    const char *pers = "eth_tls_v4";
    ret = mbedtls_ctr_drbg_seed(ctr_drbg, mbedtls_entropy_func, entropy,
                                (const unsigned char *)pers, strlen(pers));
    if (ret != 0)
    {
        TLS_LOG("[TLS] RNG seed fail: -0x%04X\n", (unsigned)(-ret));
        goto cleanup;
    }

    mbedtls_ssl_config_defaults(conf, MBEDTLS_SSL_IS_CLIENT,
                                MBEDTLS_SSL_TRANSPORT_STREAM,
                                MBEDTLS_SSL_PRESET_DEFAULT);

    mbedtls_ssl_conf_authmode(conf, MBEDTLS_SSL_VERIFY_NONE);
    mbedtls_ssl_conf_rng(conf, mbedtls_ctr_drbg_random, ctr_drbg);

    mbedtls_ssl_conf_ciphersuites(conf, PREFERRED_CIPHERS);

    mbedtls_ssl_conf_session_tickets(conf, MBEDTLS_SSL_SESSION_TICKETS_DISABLED);

    ret = mbedtls_ssl_setup(ssl, conf);
    if (ret != 0)
    {
        TLS_LOG("[TLS] Setup fail: -0x%04X\n", (unsigned)(-ret));
        goto cleanup;
    }

    ret = mbedtls_ssl_set_hostname(ssl, host);
    if (ret != 0)
    {
        TLS_LOG("[TLS] Hostname fail: -0x%04X\n", (unsigned)(-ret));
        goto cleanup;
    }

    {
        TLS_LOG("[TCP] Connecting...");
        bool connected = false;

        for (int attempt = 0; attempt <= TCP_CONNECT_RETRIES && !connected; ++attempt)
        {
            esp_task_wdt_reset();
            if (ethClient.connect(host, 443))
            {
                connected = true;
            }
            else if (attempt < TCP_CONNECT_RETRIES)
            {
                TLS_LOG(".");
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }

        if (!connected)
        {
            TLS_LOG(" FAIL\n");
            ret = -1;
            goto cleanup;
        }
        TLS_LOG(" OK\n");
    }

    {
        mbedtls_ssl_set_bio(ssl, &ethClient, eth_ssl_send, eth_ssl_recv, NULL);
        TLS_LOG("[TLS] Handshake...");

        unsigned long hs_start = millis();
        int hs_iter = 0;

        while ((ret = mbedtls_ssl_handshake(ssl)) != 0)
        {
            if (ret != MBEDTLS_ERR_SSL_WANT_READ &&
                ret != MBEDTLS_ERR_SSL_WANT_WRITE)
            {
                TLS_LOG(" FAIL (-0x%04X)\n", (unsigned)(-ret));
                goto cleanup;
            }

            if (millis() - hs_start > TLS_HANDSHAKE_TIMEOUT)
            {
                TLS_LOG(" TIMEOUT\n");
                ret = -1;
                goto cleanup;
            }

            if ((++hs_iter % 5) == 0)
                esp_task_wdt_reset();

            vTaskDelay(pdMS_TO_TICKS(2));
        }
        TLS_LOG(" OK (%lums)\n", millis() - hs_start);
    }

    {
        String request = buildHttpRequest(host, path, data, username, password);
        if (request.length() == 0)
        {
            TLS_LOG("[TLS] Request build fail\n");
            ret = -1;
            goto cleanup;
        }

        if (!sendRequest(ssl, request, TLS_WRITE_TIMEOUT))
        {
            ret = -1;
            goto cleanup;
        }
    }

    {
        unsigned char buf[SSL_BUFFER_SIZE];
        String body;
        body.reserve(256);

        unsigned long read_t = millis();
        int httpCode = 0;
        bool headerDone = false;
        bool done = false;

        String headerBuf;
        headerBuf.reserve(256);

        TLS_LOG("[HTTPS] Reading...");

        while (!done && (millis() - read_t < TLS_READ_TIMEOUT))
        {
            ret = mbedtls_ssl_read(ssl, buf, sizeof(buf) - 1);

            if (ret > 0)
            {
                buf[ret] = '\0';
                const char *chunk = reinterpret_cast<const char *>(buf);

                if (!headerDone)
                {

                    headerBuf.concat(chunk, (unsigned int)ret);

                    int sepIdx = headerBuf.indexOf("\r\n\r\n");
                    if (sepIdx >= 0)
                    {
                        headerDone = true;

                        int httpIdx = headerBuf.indexOf("HTTP/");
                        if (httpIdx >= 0)
                        {
                            int spaceIdx = headerBuf.indexOf(' ', httpIdx);
                            if (spaceIdx >= 0)
                                httpCode = headerBuf.substring(spaceIdx + 1, spaceIdx + 4).toInt();
                        }

                        int bodyOffset = sepIdx + 4;
                        if (bodyOffset < (int)headerBuf.length())
                            body.concat(headerBuf.c_str() + bodyOffset,
                                        (unsigned int)(headerBuf.length() - bodyOffset));

                        headerBuf = String();
                    }
                }
                else
                {

                    body.concat(chunk, (unsigned int)ret);
                }

                if (headerDone && body.indexOf("\"isSuccess\"") >= 0 && body.indexOf('}') >= 0)
                {
                    done = true;
                }
            }
            else if (ret == MBEDTLS_ERR_SSL_WANT_READ ||
                     ret == MBEDTLS_ERR_SSL_WANT_WRITE)
            {
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            else if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY || ret == 0)
            {

                break;
            }
            else
            {
                TLS_LOG(" [read err -0x%04X]", (unsigned)(-ret));
                break;
            }

            esp_task_wdt_reset();
        }

        TLS_LOG(" OK (%lums, hdr+%u bytes body)\n",
                millis() - read_t, (unsigned)body.length());
        TLS_LOG("[HTTP] Status: %d\n", httpCode);

        int jsonStart = body.indexOf('{');
        int jsonEnd = body.lastIndexOf('}');
        ret = -1;

        if (jsonStart >= 0 && jsonEnd > jsonStart)
        {
            bool isSuccess = parseJsonResponse(
                body.c_str() + jsonStart,
                (size_t)(jsonEnd - jsonStart + 1));

#ifdef MBEDTLS_DEBUG_ENABLE
            {
                int msgIdx = body.indexOf("\"message\"", jsonStart);
                if (msgIdx >= 0)
                {
                    int qStart = body.indexOf('"', msgIdx + 9);
                    int qStart2 = (qStart >= 0) ? body.indexOf('"', qStart + 1) : -1;
                    int qEnd = (qStart2 >= 0) ? body.indexOf('"', qStart2 + 1) : -1;
                    if (qStart2 >= 0 && qEnd > qStart2)
                        TLS_LOG("[Response] %s: %s\n",
                                isSuccess ? "OK" : "FAIL",
                                body.substring(qStart2 + 1, qEnd).c_str());
                    else
                        TLS_LOG("[Response] %s\n", isSuccess ? "OK" : "FAIL");
                }
            }
#endif

            ret = isSuccess ? 0 : -1;
        }
        else
        {

            ret = (httpCode == 200) ? 0 : -1;
        }
    }

cleanup:
    mbedtls_ssl_close_notify(ssl);
    ethClient.stop();

    mbedtls_ssl_free(ssl);
    mbedtls_ssl_config_free(conf);
    mbedtls_x509_crt_free(cacert);
    mbedtls_ctr_drbg_free(ctr_drbg);
    mbedtls_entropy_free(entropy);

    delete ssl;
    delete conf;
    delete cacert;
    delete ctr_drbg;
    delete entropy;

    TLS_LOG("[HTTPS] Closed\n\n");
    return (ret == 0) ? 200 : -1;
}

#endif