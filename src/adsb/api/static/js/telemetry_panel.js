/**
 * Aircraft Telemetry Details Slide-Out Drawer Controller.
 */

export class TelemetryPanel {
  constructor() {
    this.drawer = document.getElementById("telemetry-drawer");
    this.closeBtn = document.getElementById("drawer-close-btn");

    this.callsignEl = document.getElementById("drawer-callsign");
    this.icaoEl = document.getElementById("drawer-icao");
    this.stateEl = document.getElementById("drawer-state");

    this.squawkEl = document.getElementById("metric-squawk");
    this.altBaroEl = document.getElementById("metric-alt-baro");
    this.altGeomEl = document.getElementById("metric-alt-geom");
    this.gsEl = document.getElementById("metric-gs");
    this.trackEl = document.getElementById("metric-track");
    this.vrEl = document.getElementById("metric-vr");
    this.latlonEl = document.getElementById("metric-latlon");
    this.turnProbEl = document.getElementById("metric-turn-prob");
    this.lastSeenEl = document.getElementById("metric-last-seen");
    this.anomaliesEl = document.getElementById("drawer-anomalies");

    this.selectedIcaoHex = null;

    if (this.closeBtn) {
      this.closeBtn.addEventListener("click", () => this.close());
    }
  }

  showTrack(track) {
    if (!track) return;
    this.selectedIcaoHex = track.icao_hex || track.icao24?.toString(16).toUpperCase();

    if (this.callsignEl) {
      this.callsignEl.textContent = track.callsign ? track.callsign.trim() : "UNKNOWN";
    }
    if (this.icaoEl) {
      this.icaoEl.textContent = `0x${this.selectedIcaoHex}`;
    }

    if (this.stateEl) {
      const state = (track.state || "TENTATIVE").toLowerCase();
      this.stateEl.className = `state-pill ${state}`;
      this.stateEl.textContent = (track.state || "TENTATIVE").toUpperCase();
    }

    if (this.squawkEl) {
      this.squawkEl.textContent = track.squawk || "1200";
    }

    if (this.altBaroEl) {
      this.altBaroEl.textContent =
        track.alt_baro_ft !== null && track.alt_baro_ft !== undefined
          ? `${Math.round(track.alt_baro_ft).toLocaleString()} ft`
          : "--- ft";
    }

    if (this.altGeomEl) {
      this.altGeomEl.textContent =
        track.alt_geom_ft !== null && track.alt_geom_ft !== undefined
          ? `${Math.round(track.alt_geom_ft).toLocaleString()} ft`
          : "--- ft";
    }

    if (this.gsEl) {
      this.gsEl.textContent =
        track.groundspeed_kt !== null && track.groundspeed_kt !== undefined
          ? `${Math.round(track.groundspeed_kt)} kt`
          : "--- kt";
    }

    if (this.trackEl) {
      this.trackEl.textContent =
        track.track_deg !== null && track.track_deg !== undefined
          ? `${Math.round(track.track_deg)}°`
          : "---°";
    }

    if (this.vrEl) {
      if (track.vertical_rate_fpm !== null && track.vertical_rate_fpm !== undefined) {
        const sign = track.vertical_rate_fpm > 0 ? "+" : "";
        this.vrEl.textContent = `${sign}${Math.round(track.vertical_rate_fpm)} fpm`;
      } else {
        this.vrEl.textContent = "--- fpm";
      }
    }

    if (this.latlonEl) {
      const lat = track.lat_deg ?? track.lat;
      const lon = track.lon_deg ?? track.lon;
      if (lat !== null && lat !== undefined && lon !== null && lon !== undefined) {
        this.latlonEl.textContent = `${lat.toFixed(4)}, ${lon.toFixed(4)}`;
      } else {
        this.latlonEl.textContent = "--.----, --.----";
      }
    }

    if (this.turnProbEl) {
      const prob = track.mode_probability_turn !== undefined ? track.mode_probability_turn * 100 : 0;
      this.turnProbEl.textContent = `${Math.round(prob)} %`;
    }

    if (this.lastSeenEl) {
      this.lastSeenEl.textContent =
        track.last_seen_s !== undefined ? `${track.last_seen_s.toFixed(1)} s` : "-- s";
    }

    if (this.anomaliesEl) {
      this.anomaliesEl.innerHTML = "";
      const anomalies = track.anomalies || [];
      if (anomalies.length === 0) {
        this.anomaliesEl.innerHTML = '<span class="no-anomalies">None detected</span>';
      } else {
        anomalies.forEach((anom) => {
          const badge = document.createElement("span");
          badge.className = "anomaly-badge";
          badge.textContent = anom;
          this.anomaliesEl.appendChild(badge);
        });
      }
    }

    if (this.drawer) {
      this.drawer.classList.remove("closed");
    }
  }

  updateIfSelected(tracks) {
    if (!this.selectedIcaoHex || !tracks) return;
    const current = tracks.find(
      (t) => (t.icao_hex || t.icao24?.toString(16).toUpperCase()) === this.selectedIcaoHex
    );
    if (current) {
      this.showTrack(current);
    }
  }

  close() {
    this.selectedIcaoHex = null;
    if (this.drawer) {
      this.drawer.classList.add("closed");
    }
  }
}
