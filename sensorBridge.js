/**
 * KineFlexSensorBridge
 * Web Serial API bridge for ESP32 RX (V4.1+) <-> smart-rehab-bed CAD Simulator
 * - Works over HTTPS (GitHub Pages) on Chrome / Edge without switching Wi-Fi
 * - Receives 20 Hz JSON lines prefixed with "TELEMETRY:" at 115200 baud
 * - Sends START / STOP / CALIBRATE / PING commands (500 ms heartbeat to avoid BROWSER_TIMEOUT)
 */
class KineFlexSensorBridge {
  constructor(onTelemetry, onStatusChange = null) {
    this.onTelemetry = onTelemetry;
    this.onStatusChange = onStatusChange;
    this.port = null;
    this.writer = null;
    this.reader = null;
    this.readableStreamClosed = null;
    this.writableStreamClosed = null;
    this.pingTimer = null;
    this.connected = false;
  }

  _emitStatus(state, message = '') {
    if (typeof this.onStatusChange === 'function') {
      this.onStatusChange({ state, message, connected: this.connected });
    }
  }

  async connect() {
    if (!('serial' in navigator)) {
      const msg = 'เบราว์เซอร์นี้ไม่รองรับ Web Serial API กรุณาเปิดด้วย Google Chrome หรือ Microsoft Edge บนคอมพิวเตอร์';
      alert(msg);
      this._emitStatus('unsupported', msg);
      return false;
    }

    if (this.connected) {
      return true;
    }

    try {
      this._emitStatus('connecting', 'กำลังเลือกพอร์ต USB ของบอร์ด ESP32 RX...');
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: 115200 });
      this.connected = true;

      const textEncoder = new TextEncoderStream();
      this.writableStreamClosed = textEncoder.readable.pipeTo(this.port.writable);
      this.writer = textEncoder.writable.getWriter();

      // Send PING every 500 ms so the RX board stays alive and does not trip BROWSER_TIMEOUT
      this.pingTimer = setInterval(() => {
        this.sendCommand('PING').catch(() => {});
      }, 500);

      // Handle unexpected USB unplug
      if (this.port && typeof this.port.addEventListener === 'function') {
        this._onDisconnectEvent = () => {
          this.disconnect('สาย USB ถูกถอดออก');
        };
        this.port.addEventListener('disconnect', this._onDisconnectEvent, { once: true });
      }

      this._emitStatus('connected', 'เชื่อมต่อบอร์ด ESP32 RX ผ่าน USB Serial สำเร็จ (115200 baud)');
      this.readLoop();
      return true;
    } catch (err) {
      // User cancelled the port picker dialog
      if (err && err.name === 'NotFoundError') {
        this._emitStatus('disconnected', 'ยกเลิกการเลือกพอร์ต USB');
        return false;
      }
      const errMsg = `เชื่อมต่อไม่สำเร็จ: ${err && err.message ? err.message : err}`;
      this._emitStatus('error', errMsg);
      await this.disconnect();
      return false;
    }
  }

  async disconnect(reason = 'ตัดการเชื่อมต่อ USB Serial แล้ว') {
    const wasConnected = this.connected;
    this.connected = false;

    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }

    try {
      if (this.reader) {
        await this.reader.cancel().catch(() => {});
      }
      if (this.readableStreamClosed) {
        await this.readableStreamClosed.catch(() => {});
      }
    } catch (_) {}

    try {
      if (this.writer) {
        await this.writer.close().catch(() => {});
      }
      if (this.writableStreamClosed) {
        await this.writableStreamClosed.catch(() => {});
      }
    } catch (_) {}

    try {
      if (this.port) {
        if (this._onDisconnectEvent && typeof this.port.removeEventListener === 'function') {
          this.port.removeEventListener('disconnect', this._onDisconnectEvent);
        }
        await this.port.close().catch(() => {});
      }
    } catch (_) {}

    this.reader = null;
    this.writer = null;
    this.readableStreamClosed = null;
    this.writableStreamClosed = null;
    this.port = null;

    if (wasConnected) {
      this._emitStatus('disconnected', reason);
    }
  }

  async sendCommand(cmd) {
    if (!this.connected || !this.writer) return false;
    try {
      await this.writer.write(String(cmd).trim() + '\n');
      return true;
    } catch (err) {
      console.warn('[KineFlexSensorBridge] sendCommand failed:', err);
      return false;
    }
  }

  async readLoop() {
    const decoder = new TextDecoderStream();
    this.readableStreamClosed = this.port.readable.pipeTo(decoder.writable);
    this.reader = decoder.readable.getReader();
    let buffer = '';

    try {
      while (this.connected) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (!value) continue;

        buffer += value;
        const lines = buffer.split('\n');
        buffer = lines.pop();

        for (const rawLine of lines) {
          const line = rawLine.trim();
          if (line.startsWith('TELEMETRY:')) {
            try {
              const data = JSON.parse(line.slice('TELEMETRY:'.length));
              if (typeof this.onTelemetry === 'function') {
                this.onTelemetry(this.mapToSimulation(data));
              }
            } catch (_) {
              // Ignore partial or malformed serial line
            }
          }
        }
      }
    } catch (err) {
      if (this.connected) {
        console.warn('[KineFlexSensorBridge] readLoop error:', err);
        this.disconnect('การเชื่อมต่อ USB ขาดหาย');
      }
    } finally {
      try {
        if (this.reader) this.reader.releaseLock();
      } catch (_) {}
    }
  }

  // แปลงค่าเซนเซอร์จริงจาก ESP32 RX เป็นตัวแปรสำหรับหน้า CAD Simulation
  mapToSimulation(d) {
    const mode = typeof d.mode === 'string' ? d.mode : 'stopped';
    const result = typeof d.result === 'string' ? d.result : 'OK';
    const ready = typeof d.ready === 'string' ? d.ready : 'UNKNOWN';
    const angle = Number.isFinite(Number(d.angle)) ? Number(d.angle) : 80;
    const avgMA = Number.isFinite(Number(d.avgMA)) ? Number(d.avgMA) : 0;
    const rawMA = Number.isFinite(Number(d.rawMA)) ? Number(d.rawMA) : 0;
    const volts = Number.isFinite(Number(d.volts)) ? Number(d.volts) : 0;
    const loadIndex = Number.isFinite(Number(d.loadIndex)) ? Number(d.loadIndex) : 0;
    const emg = Number.isFinite(Number(d.emg)) ? Number(d.emg) : -1;
    const emgBase = Number.isFinite(Number(d.emgBase)) ? Number(d.emgBase) : -1;
    const emgLive = Boolean(d.emgLive);

    // 1. แปลงมุมเซอร์โว (ค่าเริ่มต้น 30..100° หรือตาม minAngle..maxAngle ที่บอร์ดส่งมา) เป็นสัดส่วน 0..1
    const minAngle = Number.isFinite(Number(d.minAngle)) ? Number(d.minAngle) : 30;
    const maxAngle = Number.isFinite(Number(d.maxAngle)) ? Number(d.maxAngle) : 100;
    const angleSpan = Math.max(1, maxAngle - minAngle);
    const progress = Math.max(0, Math.min(1, (angle - minAngle) / angleSpan));
    const theta1 = 55 + progress * 40;        // ช่วงมุมก้านหลักอ้างอิง 55° - 95°
    const s2 = 150 + progress * 220;          // ช่วงสไลด์รางอ้างอิง 150 - 370 mm

    // 2. แปลงกระแส INA219 (mA) เป็นแรงมอเตอร์ AI (600 mA = 35 N / 100%)
    const currentAbs = Math.abs(avgMA);
    const forcePct = Math.min(100, Math.round((currentAbs / 600) * 100));
    const forceN = Number(((currentAbs / 600) * 35).toFixed(1));

    // 3. ตรวจสถานะผู้ป่วยช่วยออกแรง (Assist-as-Needed) เมื่อ EMG ใกล้ emgBase และขยับปกติ
    const emgDelta = (emgLive && emg >= 0 && emgBase >= 0) ? (emg - emgBase) : 0;
    const isPatientAssisting =
      mode === 'running' &&
      emgLive &&
      emg >= 0 &&
      emgBase >= 0 &&
      Math.abs(emgDelta) <= 20;

    // 4. ตรวจสถานะฉุกเฉินจาก EMG และกระแส INA219
    const isEmergency =
      result === 'EMG_SPIKE' ||
      result === 'EMG_HIGH' ||
      result === 'OVERCURRENT_PEAK' ||
      result === 'OVERCURRENT_SUSTAINED' ||
      result.startsWith('OVERCURRENT');

    let aiStatusText = 'AI คิด: เซนเซอร์จริงทำงานปกติ กำลังประคองการเคลื่อนไหวตามวิถี';
    if (result === 'EMG_SPIKE') {
      aiStatusText = 'ฉุกเฉิน! ตรวจพบกล้ามเนื้อกระตุกผิดปกติ (EMG_SPIKE) — หยุดมอเตอร์ทันที';
    } else if (result === 'EMG_HIGH') {
      aiStatusText = 'ฉุกเฉิน! แรงเกร็งกล้ามเนื้อสูงเกินเกณฑ์ปลอดภัย (EMG_HIGH) — หยุดมอเตอร์ทันที';
    } else if (result.startsWith('OVERCURRENT')) {
      aiStatusText = `ฉุกเฉิน! ตรวจพบแรงต้านเกินพิกัดจาก INA219 (${result}) — ตัดกำลังมอเตอร์แล้ว`;
    } else if (mode === 'calibrating') {
      aiStatusText = 'AI คิด: กำลังคาริเบรตเซนเซอร์ EMG Baseline และกระแส INA219...';
    } else if (mode === 'stopped') {
      aiStatusText = 'AI คิด: ระบบหยุดพัก (รอคำสั่ง START จากแดชบอร์ด)';
    } else if (isPatientAssisting) {
      aiStatusText = `AI คิด: ผู้ป่วยกำลังช่วยออกแรงอย่างสม่ำเสมอ (Assist-as-Needed | EMG ${emg}% ใกล้ Base ${emgBase.toFixed(1)}%)`;
    }

    return {
      raw: {
        mode,
        result,
        ready,
        angle,
        emg,
        emgBase,
        emgLive,
        rawMA,
        avgMA,
        volts,
        loadIndex
      },
      progress,
      isRunning: mode === 'running',
      isStopped: mode === 'stopped',
      isEmergency,
      isPatientAssisting,
      theta1,
      s2,
      forceN,
      forcePct,
      emg,
      emgBase,
      emgLive,
      aiStatusText
    };
  }
}

window.KineFlexSensorBridge = KineFlexSensorBridge;

/**
 * KineFlexCloudBridge
 * MQTT over WSS bridge for ESP32 RX <-> Mobile (iOS Safari / Android Chrome) & Desktop
 * - Works over HTTPS (GitHub Pages) on iPhone, iPad, Android, and PC without USB cable
 * - Subscribes to: kineflex/<deviceId>/telemetry
 * - Publishes to:  kineflex/<deviceId>/cmd  (START / STOP / CALIBRATE / PING)
 * - Uses the exact same telemetry JSON & mapToSimulation() as KineFlexSensorBridge
 */
class KineFlexCloudBridge {
  constructor(onTelemetry, onStatusChange = null) {
    this.onTelemetry = onTelemetry;
    this.onStatusChange = onStatusChange;
    this.client = null;
    this.pingTimer = null;
    this.connected = false;
    this.deviceId = 'kineflex-rx-01';
    this.brokerUrl = 'wss://broker.emqx.io:8084/mqtt';
    this._mapper = new KineFlexSensorBridge();
  }

  _emitStatus(state, message = '') {
    if (typeof this.onStatusChange === 'function') {
      this.onStatusChange({
        state,
        message,
        connected: this.connected,
        transport: 'cloud',
        deviceId: this.deviceId
      });
    }
  }

  getTelemetryTopic() {
    return `kineflex/${this.deviceId}/telemetry`;
  }

  getCommandTopic() {
    return `kineflex/${this.deviceId}/cmd`;
  }

  async connect(deviceId = 'kineflex-rx-01', brokerUrl = 'wss://broker.emqx.io:8084/mqtt') {
    if (typeof window.mqtt === 'undefined') {
      const msg = 'ไม่พบไลบรารี MQTT.js กรุณาตรวจสอบการเชื่อมต่ออินเทอร์เน็ตแล้วลองรีเฟรชหน้าเว็บ';
      alert(msg);
      this._emitStatus('error', msg);
      return false;
    }

    if (this.connected) {
      await this.disconnect();
    }

    const cleanId = String(deviceId || 'kineflex-rx-01').trim().replace(/[^a-zA-Z0-9_-]/g, '');
    this.deviceId = cleanId || 'kineflex-rx-01';
    this.brokerUrl = String(brokerUrl || 'wss://broker.emqx.io:8084/mqtt').trim();

    this._emitStatus('connecting', `กำลังเชื่อมต่อ Cloud MQTT (${this.deviceId})...`);

    return new Promise((resolve) => {
      let settled = false;
      const clientId = `kineflex-web-${Math.random().toString(16).slice(2, 10)}`;

      try {
        this.client = window.mqtt.connect(this.brokerUrl, {
          clientId,
          clean: true,
          connectTimeout: 8000,
          reconnectPeriod: 3000
        });
      } catch (err) {
        const msg = `เริ่มการเชื่อมต่อ Cloud ไม่สำเร็จ: ${err && err.message ? err.message : err}`;
        this._emitStatus('error', msg);
        resolve(false);
        return;
      }

      this.client.on('connect', () => {
        this.connected = true;
        const topic = this.getTelemetryTopic();
        this.client.subscribe(topic, { qos: 0 }, (err) => {
          if (err) {
            this._emitStatus('error', `Subscribe ไม่สำเร็จ: ${err.message}`);
            return;
          }
          this._emitStatus('connected', `เชื่อมต่อ Cloud MQTT สำเร็จ — รอรับข้อมูลจากบอร์ด "${this.deviceId}"`);
        });

        if (this.pingTimer) clearInterval(this.pingTimer);
        this.pingTimer = setInterval(() => {
          this.sendCommand('PING');
        }, 500);

        if (!settled) {
          settled = true;
          resolve(true);
        }
      });

      this.client.on('message', (_topic, payload) => {
        try {
          let text = typeof payload === 'string' ? payload : payload.toString('utf8');
          text = text.trim();
          if (text.startsWith('TELEMETRY:')) {
            text = text.slice('TELEMETRY:'.length);
          }
          const data = JSON.parse(text);
          if (typeof this.onTelemetry === 'function') {
            this.onTelemetry(this._mapper.mapToSimulation(data));
          }
        } catch (_) {
          // Ignore malformed JSON payloads
        }
      });

      this.client.on('reconnect', () => {
        this._emitStatus('connecting', `กำลังเชื่อมต่อ Cloud MQTT ใหม่ (${this.deviceId})...`);
      });

      this.client.on('error', (err) => {
        const msg = `Cloud MQTT Error: ${err && err.message ? err.message : err}`;
        this._emitStatus('error', msg);
        if (!settled) {
          settled = true;
          resolve(false);
        }
      });

      this.client.on('close', () => {
        if (this.connected) {
          this.connected = false;
        }
      });
    });
  }

  async disconnect(reason = 'ตัดการเชื่อมต่อ Cloud MQTT แล้ว') {
    const wasConnected = this.connected || Boolean(this.client);
    this.connected = false;

    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }

    if (this.client) {
      try {
        this.client.end(true);
      } catch (_) {}
      this.client = null;
    }

    if (wasConnected) {
      this._emitStatus('disconnected', reason);
    }
  }

  async sendCommand(cmd) {
    if (!this.connected || !this.client) return false;
    try {
      this.client.publish(this.getCommandTopic(), String(cmd).trim(), { qos: 0 });
      return true;
    } catch (err) {
      console.warn('[KineFlexCloudBridge] sendCommand failed:', err);
      return false;
    }
  }
}

window.KineFlexCloudBridge = KineFlexCloudBridge;

