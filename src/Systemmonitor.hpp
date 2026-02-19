#ifndef SYSTEM_MONITOR_HPP
#define SYSTEM_MONITOR_HPP

#include <Arduino.h>
#include "esp_system.h"
#include "esp_heap_caps.h"

class SystemMonitor
{
public:
    SystemMonitor() {}

    void printCurrent()
    {
        uint32_t freeHeap = ESP.getFreeHeap();
        uint32_t totalHeap = ESP.getHeapSize();
        uint32_t minFreeHeap = ESP.getMinFreeHeap();

        uint8_t heapPct = (totalHeap > 0) ? ((totalHeap - freeHeap) * 100) / totalHeap : 0;

        Serial.println(F("\n--- [ SYS MONITOR ] ---"));

        Serial.printf("HEAP : %u / %u bytes (%u%% used)\n", freeHeap, totalHeap, heapPct);
        Serial.printf("MIN  : %u bytes (Lowest)\n", minFreeHeap);

        if (ESP.getPsramSize() > 0)
        {
            Serial.printf("PSRAM: %u / %u bytes\n", ESP.getFreePsram(), ESP.getPsramSize());
        }

        Serial.printf("TASK : %u active\n", uxTaskGetNumberOfTasks());
        Serial.printf("UP   : %lu sec\n", millis() / 1000);
        Serial.println(F("-----------------------"));
    }

    void checkAndPrintWarnings()
    {

        if (ESP.getFreeHeap() < 10240)
        {
            Serial.printf("⚠️ LOW MEMORY: %u bytes left!\n", ESP.getFreeHeap());
        }

        UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(NULL);
        if (stackLeft < 500)
        {
            Serial.printf("⚠️ LOW STACK: %u bytes left!\n", stackLeft);
        }
    }

    void begin()
    {
    }
};

#endif