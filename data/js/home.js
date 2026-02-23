document.addEventListener("DOMContentLoaded", () => {
  let chartT = null;
  let chartS = null;
  let firstRun = true;

  // --- 1. REFERENSI ELEMENT (Load secepat mungkin) ---
  const networkMode = document.getElementById('networkMode');
  const ssid = document.getElementById('ssid');
  const ipAddres = document.getElementById('ipAddres');
  const macAddress = document.getElementById('macAddress');
  const sendInterval = document.getElementById('sendInterval');
  const protocolMode = document.getElementById('protocolMode');
  const endpoint = document.getElementById('endpoint');
  const connStatus = document.getElementById('connStatus');
  const jobNumber = document.getElementById('jobNumber');

  // Disable inputs (Visual Read-only)
  [networkMode, ssid, ipAddres, macAddress, jobNumber,
    protocolMode, endpoint, connStatus, sendInterval]
  .forEach(el => {
    if (el) el.disabled = true;
  });

  // [UPDATED] Analog Input Elements Reference (Hanya Scaled Value)
  const aiScaledInputs = [
    document.getElementById('ai1Scaled'),
    document.getElementById('ai2Scaled'),
    document.getElementById('ai3Scaled'),
    document.getElementById('ai4Scaled')
  ];

  // Digital I/O Elements
  const diValues = [
    document.getElementById('di1Value'),
    document.getElementById('di2Value'),
    document.getElementById('di3Value'),
    document.getElementById('di4Value'),
  ];
  const diTexts = [
    document.getElementById('di1Text'),
    document.getElementById('di2Text'),
    document.getElementById('di3Text'),
    document.getElementById('di4Text'),
  ];
  const diUnits = [
    document.getElementById('di1Unit'),
    document.getElementById('di2Unit'),
    document.getElementById('di3Unit'),
    document.getElementById('di4Unit'),
  ];

  // --- 2. FUNGSI FETCH DATA (Polling) ---
  function getSensorData() {
    fetch('/homeLoad', {
        method: 'GET'
      })
      .then(r => r.ok ? r.json() : Promise.reject(new Error(`HTTP ${r.status}`)))
      .then(data => {
        // A. Update Network Info
        if (networkMode) networkMode.value = data.networkMode || '-';
        if (ssid) ssid.value = data.ssid || '-';
        if (ipAddres) ipAddres.value = data.ipAddress || '-';
        if (sendInterval) sendInterval.value = data.sendInterval || '-';
        if (protocolMode) protocolMode.value = data.protocolMode || '-';
        if (endpoint) endpoint.value = data.endpoint || '-';
        if (macAddress) macAddress.value = data.macAddress || '-';
        if (connStatus) connStatus.value = data.connStatus || '-';
        if (jobNumber) jobNumber.value = data.jobNumber || '-';

        // B. Parse Sensor Data
        const DI_values = (data.DI && data.DI.value) || [0, 0, 0, 0];
        const DI_taskmode = (data.DI && data.DI.taskMode) || ['Normal', 'Counting', 'Cycle Time', 'Run Time'];

        const AI_rawValues = (data.AI && data.AI.rawValue) || [0, 0, 0, 0];
        const AI_scaledValues = (data.AI && data.AI.scaledValue) || [0, 0, 0, 0];
        const enabled_AI = Array.isArray(data.enAI) ? data.enAI : [1, 1, 1, 1];

        // [UPDATED] Update Analog Input Status (Hanya Scaled Value)
        for (let i = 0; i < 4; i++) {
          if (aiScaledInputs[i]) {
            // Tampilkan Scaled Value dengan 2 angka di belakang koma
            aiScaledInputs[i].value = AI_scaledValues[i] !== undefined ? parseFloat(AI_scaledValues[i]).toFixed(2) : '0.00';
          }
        }

        // C. Update Digital Input UI
        for (let i = 0; i < 4; i++) {
          if (diValues[i]) diValues[i].value = DI_values[i] ?? '0';
          if (diTexts[i]) diTexts[i].textContent = DI_taskmode[i] ?? '-';
          if (diUnits[i]) {
            const mode = DI_taskmode[i];
            diUnits[i].textContent =
              mode === 'Counting' ? 'pcs' :
              mode === 'Cycle Time' ? 'sec' :
              mode === 'Run Time' ? 'min' :
              mode === 'Pulse Mode' ? 'Hz' : '.';
          }
        }

        // --- D. Update Grafik (HANYA JIKA HIGHCHARTS SUDAH LOAD) ---
        /* if (chartT && chartS) {
          // 1. Atur Visibilitas (Sekali saja saat awal)
          if (firstRun) {
            for (let i = 0; i < enabled_AI.length && i < 4; i++) {
              const isVisible = enabled_AI[i] === 1;
              if (chartT.series[i]) chartT.series[i].setVisible(isVisible, false);
              if (chartS.series[i]) chartS.series[i].setVisible(isVisible, false);
            }
            chartT.redraw();
            chartS.redraw();
            firstRun = false;
          }

          // 2. Tentukan Waktu X-Axis (Gunakan Waktu Browser agar Grafik Mulus)
          let x = (new Date()).getTime();

          // 3. Plot Data Raw
          const sensorRawValues = AI_rawValues.map(v => Math.round((v || 0) * 100) / 100);
          sensorRawValues.forEach((y, i) => {
            if (enabled_AI[i] === 1 && chartT.series[i]) {
              const shift = chartT.series[i].data.length > 300;
              chartT.series[i].addPoint([x, y], false, shift, true);
            }
          });
          chartT.redraw();

          // 4. Plot Data Scaled
          const sensorScaledValues = AI_scaledValues.map(v => Math.round((v || 0) * 100) / 100);
          sensorScaledValues.forEach((y, i) => {
            if (enabled_AI[i] === 1 && chartS.series[i]) {
              const shift = chartS.series[i].data.length > 300;
              chartS.series[i].addPoint([x, y], false, shift, true);
            }
          });
          chartS.redraw();
        } // <--- INI YANG HILANG SEBELUMNYA */
      })
      .catch(err => console.error('Data fetch error:', err));
  }

  // --- 3. FUNGSI LAZY LOAD HIGHCHARTS ---
  function loadHighcharts(callback) {
    if (typeof Highcharts !== 'undefined') {
      callback();
      return;
    }

    const script = document.createElement('script');
    script.src = 'js/highcharts.js';
    script.onload = callback;
    script.onerror = () => {
      console.warn("Local Highcharts failed, trying CDN...");
      const cdnScript = document.createElement('script');
      cdnScript.src = 'https://code.highcharts.com/highcharts.js';
      cdnScript.onload = callback;
      document.head.appendChild(cdnScript);
    };
    document.head.appendChild(script);
  }

  // --- 4. MAIN EXECUTION ---
  getSensorData();
  setInterval(getSensorData, 1000);

  /* loadHighcharts(() => {
    console.log("Highcharts Loaded!");

    chartT = new Highcharts.Chart({
      chart: {
        renderTo: 'chartSensor',
        type: 'line'
      },
      title: {
        text: 'Analog Input Raw Value'
      },
      series: [{
          name: 'AI 1: ',
          data: [],
          color: '#ef4444'
        },
        {
          name: 'AI 2: ',
          data: [],
          color: '#22c55e'
        },
        {
          name: 'AI 3: ',
          data: [],
          color: '#3b82f6'
        },
        {
          name: 'AI 4: ',
          data: [],
          color: '#eab308'
        },
      ],
      plotOptions: {
        line: {
          animation: false,
          dataLabels: {
            enabled: false
          }
        }
      },
      xAxis: {
        type: 'datetime',
        dateTimeLabelFormats: {
          second: '%H:%M:%S'
        }
      },
      yAxis: {
        title: {
          text: 'ADC Value'
        }
      },
      credits: {
        enabled: false
      },
      legend: {
        labelFormatter: function() {
          const pt = this.data.length ? this.data[this.data.length - 1] : null;
          return this.name + (pt ? pt.y : 'N/A');
        }
      }
    });

    chartS = new Highcharts.Chart({
      chart: {
        renderTo: 'chartSensorScaled',
        type: 'line'
      },
      title: {
        text: 'Analog Input Scaled Value'
      },
      series: [{
          name: 'AI 1: ',
          data: [],
          color: '#ef4444'
        },
        {
          name: 'AI 2: ',
          data: [],
          color: '#22c55e'
        },
        {
          name: 'AI 3: ',
          data: [],
          color: '#3b82f6'
        },
        {
          name: 'AI 4: ',
          data: [],
          color: '#eab308'
        },
      ],
      plotOptions: {
        line: {
          animation: false,
          dataLabels: {
            enabled: false
          }
        }
      },
      xAxis: {
        type: 'datetime',
        dateTimeLabelFormats: {
          second: '%H:%M:%S'
        }
      },
      yAxis: {
        title: {
          text: 'Scaled Value'
        }
      },
      credits: {
        enabled: false
      },
      legend: {
        labelFormatter: function() {
          const pt = this.data.length ? this.data[this.data.length - 1] : null;
          return this.name + (pt ? pt.y : 'N/A');
        }
      }
    });
  }); */

});