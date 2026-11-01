/**
 * Reconnecting WebSocket client for ADS-B telemetry and raw wire streams.
 */

export class WebSocketClient {
  constructor({ onTelemetry, onWire, onStatusChange }) {
    this.onTelemetry = onTelemetry || (() => {});
    this.onWire = onWire || (() => {});
    this.onStatusChange = onStatusChange || (() => {});

    this.telemetryWs = null;
    this.wireWs = null;
    this.reconnectTimer = null;
    this.wireFormat = "AVR";
    this.isConnected = false;
  }

  connect() {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = window.location.host;

    this._connectTelemetry(`${protocol}//${host}/ws/telemetry`);
    this._connectWire(`${protocol}//${host}/ws/wire?format=${this.wireFormat}`);
  }

  setWireFormat(format) {
    this.wireFormat = format;
    if (this.wireWs) {
      try {
        this.wireWs.close();
      } catch (e) {
        // ignore
      }
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      const host = window.location.host;
      this._connectWire(`${protocol}//${host}/ws/wire?format=${this.wireFormat}`);
    }
  }

  _connectTelemetry(url) {
    try {
      this.telemetryWs = new WebSocket(url);

      this.telemetryWs.onopen = () => {
        this.isConnected = true;
        this.onStatusChange("CONNECTED");
      };

      this.telemetryWs.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.onTelemetry(data);
        } catch (err) {
          console.warn("Failed to parse telemetry JSON", err);
        }
      };

      this.telemetryWs.onclose = () => {
        this.isConnected = false;
        this.onStatusChange("OFFLINE");
        this._scheduleReconnect();
      };

      this.telemetryWs.onerror = () => {
        this.isConnected = false;
        this.onStatusChange("OFFLINE");
      };
    } catch (e) {
      this._scheduleReconnect();
    }
  }

  _connectWire(url) {
    try {
      this.wireWs = new WebSocket(url);

      this.wireWs.onmessage = (event) => {
        this.onWire(event.data);
      };

      this.wireWs.onclose = () => {
        // Will reconnect on overall schedule
      };

      this.wireWs.onerror = () => {
        // Handled silently
      };
    } catch (e) {
      // ignore
    }
  }

  _scheduleReconnect() {
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, 2000);
  }
}
