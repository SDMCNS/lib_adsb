/**
 * Leaflet Tactical Map Fallback & Dual Display Controller.
 */

export class LeafletMap {
  constructor(mapContainerId, { onSelectTrack } = {}) {
    this.containerId = mapContainerId;
    this.onSelectTrack = onSelectTrack || (() => { });
    this.map = null;
    this.markers = new Map(); // icao -> marker

    this.receiverMarkers = new Map(); // rx_id -> marker
    this.coverageCircles = new Map(); // rx_id -> circle
    this.jammerMarkers = new Map(); // source_id -> marker
    this.contourCircles = new Map(); // source_id -> circle

    this.defaultCenter = [52.3105, 4.7683];
    this.defaultZoom = 8;

    this._initMap();
  }

  _initMap() {
    if (typeof window.L === "undefined") {
      console.info("Leaflet library not loaded; tactical canvas radar is primary display.");
      const el = document.getElementById(this.containerId);
      if (el) {
        el.innerHTML =
          '<div style="color:#64748b;padding:24px;font-family:monospace;text-align:center;">Leaflet Map offline. Canvas Radar active.</div>';
      }
      return;
    }

    try {
      this.map = window.L.map(this.containerId, {
        center: this.defaultCenter,
        zoom: this.defaultZoom,
        zoomControl: false,
      });

      // Primary tactical dark basemap (Esri World Dark Gray - 100% free, no key required)
      const esriDarkLayer = window.L.tileLayer(
        "https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}",
        {
          attribution: "Tiles &copy; Esri &mdash; Esri, DeLorme, NAVTEQ",
          maxZoom: 16,
        }
      );

      // OpenStreetMap standard fallback
      const osmLayer = window.L.tileLayer(
        "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        {
          attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
          maxZoom: 19,
        }
      );

      // Set ESRI Dark Canvas as default basemap
      esriDarkLayer.addTo(this.map);

      // Add tactical layer groups
      this.coverageLayer = window.L.layerGroup().addTo(this.map);
      this.contourLayer = window.L.layerGroup().addTo(this.map);
      this.receiverLayer = window.L.layerGroup().addTo(this.map);
      this.jammerLayer = window.L.layerGroup().addTo(this.map);

      // Add basemap layer control for easy switching in the top-right
      window.L.control.layers(
        {
          "Esri Dark Canvas": esriDarkLayer,
          "OpenStreetMap": osmLayer,
        },
        {
          "Receivers": this.receiverLayer,
          "Coverage Rings": this.coverageLayer,
          "Jammers": this.jammerLayer,
          "Hazard Contours": this.contourLayer,
        },
        { position: "topright" }
      ).addTo(this.map);
    } catch (err) {
      console.warn("Leaflet initialization warning:", err);
    }
  }

  updateTracks(tracks) {
    if (!this.map || typeof window.L === "undefined") return;

    const seenIcaos = new Set();

    (tracks || []).forEach((track) => {
      const lat = track.lat_deg ?? track.lat;
      const lon = track.lon_deg ?? track.lon;
      if (lat === null || lat === undefined || lon === null || lon === undefined) return;

      const icao = track.icao_hex || track.icao24?.toString(16).toUpperCase();
      seenIcaos.add(icao);

      const callsign = (track.callsign ? track.callsign.trim() : null) || icao;
      const alt = track.alt_baro_ft ?? track.alt_geom_ft;
      const altStr = alt !== null && alt !== undefined ? `${Math.round(alt)}ft` : "";

      if (this.markers.has(icao)) {
        const marker = this.markers.get(icao);
        marker.setLatLng([lat, lon]);
      } else {
        const customIcon = window.L.divIcon({
          className: "atc-leaflet-marker",
          html: `<div style="color:#00ff99;font-family:monospace;font-size:10px;text-shadow:0 0 4px #00ff99;">✈ <b>${callsign}</b><br>${altStr}</div>`,
          iconSize: [80, 30],
        });

        const marker = window.L.marker([lat, lon], { icon: customIcon }).addTo(this.map);
        marker.on("click", () => this.onSelectTrack(track));
        this.markers.set(icao, marker);
      }
    });

    // Remove stale markers
    for (const [icao, marker] of this.markers.entries()) {
      if (!seenIcaos.has(icao)) {
        this.map.removeLayer(marker);
        this.markers.delete(icao);
      }
    }
  }

  updateReceivers(receivers) {
    if (!this.map || typeof window.L === "undefined") return;
    const seen = new Set();

    (receivers || []).forEach((rx) => {
      const rxId = rx.rx_id;
      seen.add(rxId);
      const lat = rx.lat_deg ?? rx.lat;
      const lon = rx.lon_deg ?? rx.lon;
      if (lat === null || lat === undefined || lon === null || lon === undefined) return;

      const status = rx.status || "NORMAL";
      const statusClass = `status-${status.toLowerCase()}`;
      const sinr = rx.metrics ? rx.metrics.sinr_db.toFixed(1) : "--";
      const ringColor = status === "JAMMED" ? "#ef4444" : status === "DEGRADED" ? "#f59e0b" : "#00ff99";

      if (this.receiverMarkers.has(rxId)) {
        const m = this.receiverMarkers.get(rxId);
        m.setLatLng([lat, lon]);
        const container = m.getElement()?.querySelector(".rx-marker-container");
        if (container) {
          container.className = `rx-marker-container ${statusClass}`;
          const valSpan = container.querySelector(".sinr-val");
          if (valSpan) valSpan.textContent = `${sinr}dB`;
        }

        if (this.coverageCircles.has(rxId)) {
          const circle = this.coverageCircles.get(rxId);
          circle.setLatLng([lat, lon]);
          circle.setStyle({ color: ringColor });
        }
      } else {
        const icon = window.L.divIcon({
          className: "atc-rx-marker",
          html: `
            <div class="rx-marker-container ${statusClass}">
              <span>🗼</span>
              <b>${rx.name || rxId}</b>
              <span class="sinr-val" style="font-size:9px;opacity:0.8;">${sinr}dB</span>
            </div>
          `,
          iconSize: [140, 28],
          iconAnchor: [70, 14],
        });
        const marker = window.L.marker([lat, lon], { icon }).addTo(this.receiverLayer);
        marker.bindPopup(`
          <div style="font-family:monospace;font-size:11px;color:#0a0e17;min-width:180px;">
            <b>${rx.name || rxId} (${rxId})</b><br>
            Status: <b>${status}</b><br>
            SINR: <b>${sinr} dB</b><br>
            Sensitivity: ${rx.sensitivity_dbm} dBm<br>
            Antenna Gain: +${rx.antenna_gain_peak_dbi} dBi<br>
            Noise Floor: ${rx.noise_floor_dbm} dBm<br>
            <button onclick="window.deleteReceiver('${rxId}')" style="margin-top:8px;width:100%;background:#ef4444;color:white;border:none;padding:4px 8px;border-radius:3px;cursor:pointer;font-weight:bold;">DELETE STATION</button>
          </div>
        `);
        this.receiverMarkers.set(rxId, marker);

        const coverageRadiusM = (rx.coverage_radius_km || 250) * 1000;
        const circle = window.L.circle([lat, lon], {
          radius: coverageRadiusM,
          color: ringColor,
          weight: 1,
          dashArray: "4, 6",
          fillOpacity: 0.02,
        }).addTo(this.coverageLayer);
        this.coverageCircles.set(rxId, circle);
      }
    });

    for (const [id, marker] of this.receiverMarkers.entries()) {
      if (!seen.has(id)) {
        this.receiverLayer.removeLayer(marker);
        this.receiverMarkers.delete(id);
        if (this.coverageCircles.has(id)) {
          this.coverageLayer.removeLayer(this.coverageCircles.get(id));
          this.coverageCircles.delete(id);
        }
      }
    }
  }

  updateInterference(sources) {
    if (!this.map || typeof window.L === "undefined") return;
    const seen = new Set();

    (sources || []).forEach((src) => {
      const srcId = src.source_id;
      seen.add(srcId);
      const lat = src.lat_deg ?? src.lat;
      const lon = src.lon_deg ?? src.lon;
      if (lat === null || lat === undefined || lon === null || lon === undefined) return;

      const isActive = src.is_active;

      if (this.jammerMarkers.has(srcId)) {
        const m = this.jammerMarkers.get(srcId);
        m.setLatLng([lat, lon]);
        const container = m.getElement()?.querySelector(".jammer-marker-container");
        if (container) {
          container.style.opacity = isActive ? "1.0" : "0.45";
        }

        if (this.contourCircles.has(srcId)) {
          const contour = this.contourCircles.get(srcId);
          contour.setLatLng([lat, lon]);
          contour.setStyle({
            fillOpacity: isActive ? 0.12 : 0.0,
            strokeOpacity: isActive ? 0.9 : 0.0,
          });
        }
      } else {
        const icon = window.L.divIcon({
          className: "atc-jammer-marker",
          html: `
            <div class="jammer-marker-container" style="opacity:${isActive ? "1.0" : "0.45"};">
              <span style="color:#ef4444;font-size:12px;">☢</span>
              <b style="color:#ef4444;">${src.name || srcId}</b>
              <span style="font-size:9px;color:#fca5a5;">${src.tx_power_dbm}dBm</span>
            </div>
          `,
          iconSize: [140, 28],
          iconAnchor: [70, 14],
        });
        const marker = window.L.marker([lat, lon], { icon }).addTo(this.jammerLayer);
        marker.bindPopup(`
          <div style="font-family:monospace;font-size:11px;color:#0a0e17;min-width:180px;">
            <b>${src.name || srcId} (${srcId})</b><br>
            Power: <b>${src.tx_power_dbm} dBm</b> (${src.tx_power_w} W)<br>
            Frequency: ${(src.frequency_hz / 1e6).toFixed(1)} MHz<br>
            Bandwidth: ${src.bandwidth_mhz} MHz<br>
            Transmitting: <b>${isActive ? "YES (ACTIVE)" : "MUTED"}</b><br>
            <div style="display:flex;gap:4px;margin-top:8px;">
              <button onclick="window.toggleJammer('${srcId}')" style="flex:1;background:#f59e0b;color:black;border:none;padding:4px 6px;border-radius:3px;cursor:pointer;font-weight:bold;">TOGGLE</button>
              <button onclick="window.deleteJammer('${srcId}')" style="flex:1;background:#ef4444;color:white;border:none;padding:4px 6px;border-radius:3px;cursor:pointer;font-weight:bold;">DELETE</button>
            </div>
          </div>
        `);
        this.jammerMarkers.set(srcId, marker);

        const contourRadiusM = (src.effective_range_km || 40) * 1000;
        const contour = window.L.circle([lat, lon], {
          radius: contourRadiusM,
          color: "#ef4444",
          fillColor: "#ef4444",
          fillOpacity: isActive ? 0.12 : 0.0,
          strokeOpacity: isActive ? 0.9 : 0.0,
          className: "pulsing-hazard-contour",
        }).addTo(this.contourLayer);
        this.contourCircles.set(srcId, contour);
      }
    });

    for (const [id, marker] of this.jammerMarkers.entries()) {
      if (!seen.has(id)) {
        this.jammerLayer.removeLayer(marker);
        this.jammerMarkers.delete(id);
        if (this.contourCircles.has(id)) {
          this.contourLayer.removeLayer(this.contourCircles.get(id));
          this.contourCircles.delete(id);
        }
      }
    }
  }

  armCoordinatePicker(callback) {
    if (!this.map) return;
    const container = this.map.getContainer();
    container.style.cursor = "crosshair";

    const clickHandler = (e) => {
      container.style.cursor = "";
      this.map.off("click", clickHandler);
      if (callback) {
        callback(e.latlng.lat, e.latlng.lng);
      }
    };
    this.map.on("click", clickHandler);
  }

  invalidateSize() {
    if (this.map) {
      this.map.invalidateSize();
    }
  }
}
