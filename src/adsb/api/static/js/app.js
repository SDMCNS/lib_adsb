/**
 * Main Tactical Radar Web Application Coordinator.
 */

import { CanvasRadar } from "./canvas_radar.js";
import { LeafletMap } from "./leaflet_map.js";
import { TelemetryPanel } from "./telemetry_panel.js";
import { WebSocketClient } from "./websocket_client.js";

document.addEventListener("DOMContentLoaded", () => {
  // DOM Elements
  const btnPlay = document.getElementById("btn-play");
  const btnPause = document.getElementById("btn-pause");
  const btnStep = document.getElementById("btn-step");
  const btnReset = document.getElementById("btn-reset");
  const speedButtons = document.querySelectorAll(".speed-btn");
  const scenarioSelect = document.getElementById("scenario-select");

  const simTimeEl = document.getElementById("sim-time");
  const trackCountEl = document.getElementById("track-count");
  const connStatusEl = document.getElementById("conn-status");

  const btnToggleRadar = document.getElementById("btn-toggle-radar");
  const btnToggleMap = document.getElementById("btn-toggle-map");
  const radarContainer = document.getElementById("radar-container");
  const mapContainer = document.getElementById("map-container");

  const btnToggleConsole = document.getElementById("btn-toggle-console");
  const consoleBody = document.getElementById("console-body");
  const wireLogEl = document.getElementById("wire-log");
  const injectForm = document.getElementById("inject-form");
  const injectFormat = document.getElementById("inject-format");
  const injectInput = document.getElementById("inject-raw-data");
  const injectFeedback = document.getElementById("inject-feedback");
  const wireFormatTag = document.getElementById("wire-format-tag");

  // Subsystems
  const telemetryPanel = new TelemetryPanel();

  const onSelectTrack = (track) => {
    telemetryPanel.showTrack(track);
    radar.setSelectedIcao(track.icao_hex || track.icao24?.toString(16).toUpperCase());
  };

  const radar = new CanvasRadar("radar-canvas", { onSelectTrack });
  const map = new LeafletMap("leaflet-map", { onSelectTrack });

  // WebSocket Client
  const wsClient = new WebSocketClient({
    onTelemetry: (data) => {
      if (data.sim_time_s !== undefined && simTimeEl) {
        simTimeEl.textContent = `${data.sim_time_s.toFixed(1)} s`;
      }

      if (data.tracks) {
        if (trackCountEl) {
          trackCountEl.textContent = data.tracks.length;
        }
        radar.updateTracks(data.tracks);
        map.updateTracks(data.tracks);
        telemetryPanel.updateIfSelected(data.tracks);
      }

      if (data.receivers) {
        map.updateReceivers(data.receivers);
        updateSensorHudReceivers(data.receivers);
        const rxCountEl = document.getElementById("kpi-rx-count");
        if (rxCountEl) rxCountEl.textContent = data.receivers.length;
        const sensorCountEl = document.getElementById("sensor-count");
        if (sensorCountEl) sensorCountEl.textContent = data.receivers.length;
      }

      if (data.interference_sources) {
        map.updateInterference(data.interference_sources);
        updateSensorHudJammers(data.interference_sources);
        const jamCountEl = document.getElementById("kpi-jammer-count");
        if (jamCountEl) {
          const activeCount = data.interference_sources.filter((s) => s.is_active).length;
          jamCountEl.textContent = activeCount;
        }
      }

      if (data.receivers || data.interference_sources) {
        updateSpectrumHealth(data.receivers, data.interference_sources);
      }
    },
    onWire: (wireMsg) => {
      if (!wireLogEl) return;
      const line = document.createElement("div");
      line.className = "wire-log-line";
      line.textContent = `[RX] ${typeof wireMsg === "string" ? wireMsg.trim() : "[BINARY BEAST FRAME]"}`;
      wireLogEl.appendChild(line);

      // Keep max 60 log lines
      while (wireLogEl.children.length > 60) {
        wireLogEl.removeChild(wireLogEl.firstChild);
      }
      wireLogEl.scrollTop = wireLogEl.scrollHeight;
    },
    onStatusChange: (status) => {
      if (!connStatusEl) return;
      if (status === "CONNECTED") {
        connStatusEl.className = "conn-pill connected";
        connStatusEl.textContent = "CONNECTED";
      } else {
        connStatusEl.className = "conn-pill offline";
        connStatusEl.textContent = "OFFLINE";
      }
    },
  });

  wsClient.connect();

  // Simulation Controls REST API
  if (btnPlay) {
    btnPlay.addEventListener("click", async () => {
      await fetch("/api/v1/simulation/start", { method: "POST" });
    });
  }

  if (btnPause) {
    btnPause.addEventListener("click", async () => {
      await fetch("/api/v1/simulation/pause", { method: "POST" });
    });
  }

  if (btnStep) {
    btnStep.addEventListener("click", async () => {
      await fetch("/api/v1/simulation/step", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ step_s: 1.0 }),
      });
    });
  }

  if (btnReset) {
    btnReset.addEventListener("click", async () => {
      await fetch("/api/v1/simulation/reset", { method: "POST" });
      radar.updateTracks([]);
      map.updateTracks([]);
      telemetryPanel.close();
      if (simTimeEl) simTimeEl.textContent = "0.0 s";
      if (trackCountEl) trackCountEl.textContent = "0";
    });
  }

  speedButtons.forEach((btn) => {
    btn.addEventListener("click", async () => {
      const speed = parseFloat(btn.dataset.speed);
      await fetch("/api/v1/simulation/speed", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ speed_multiplier: speed }),
      });
      speedButtons.forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
    });
  });

  if (scenarioSelect) {
    scenarioSelect.addEventListener("change", async () => {
      const scenarioName = scenarioSelect.value;
      await fetch("/api/v1/simulation/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          scenario_name: scenarioName,
          speed_multiplier: 1.0,
          seed: 42,
        }),
      });
      radar.updateTracks([]);
      map.updateTracks([]);
      telemetryPanel.close();
      if (simTimeEl) simTimeEl.textContent = "0.0 s";
      if (trackCountEl) trackCountEl.textContent = "0";
    });
  }

  // View Switching (Radar vs Map)
  if (btnToggleRadar && btnToggleMap) {
    btnToggleRadar.addEventListener("click", () => {
      btnToggleRadar.classList.add("active");
      btnToggleMap.classList.remove("active");
      radarContainer.classList.add("active");
      mapContainer.classList.remove("active");
      if (typeof radar.resize === "function") {
        radar.resize();
      }
    });

    btnToggleMap.addEventListener("click", () => {
      btnToggleMap.classList.add("active");
      btnToggleRadar.classList.remove("active");
      mapContainer.classList.add("active");
      radarContainer.classList.remove("active");
      map.invalidateSize();
    });
  }

  // Wire Console Collapsible & Injection
  if (btnToggleConsole && consoleBody) {
    btnToggleConsole.addEventListener("click", () => {
      consoleBody.classList.toggle("collapsed");
      const icon = btnToggleConsole.querySelector(".toggle-icon");
      if (icon) {
        icon.textContent = consoleBody.classList.contains("collapsed") ? "▲" : "▼";
      }
    });
  }

  if (injectFormat) {
    injectFormat.addEventListener("change", () => {
      const fmt = injectFormat.value;
      if (wireFormatTag) wireFormatTag.textContent = fmt;
      wsClient.setWireFormat(fmt);

      if (fmt === "AVR") {
        injectInput.placeholder = "*8D4840D6202CC371C32CE0576098;";
      } else if (fmt === "BEAST") {
        injectInput.placeholder = "1A33... (hex) or raw bytes";
      } else {
        injectInput.placeholder = "MSG,1,1,1,4840D6,1,,,,,,,KLM1023,...";
      }
    });
  }

  if (injectForm) {
    injectForm.addEventListener("submit", async (evt) => {
      evt.preventDefault();
      const format = injectFormat.value;
      const rawData = injectInput.value.trim();
      if (!rawData) return;

      try {
        const res = await fetch("/api/v1/inject/frame", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ format, raw_data: rawData }),
        });

        if (res.ok) {
          const data = await res.json();
          injectFeedback.className = "inject-feedback success";
          injectFeedback.textContent = `Injected [${data.df}] ICAO:${data.icao_hex} Callsign:${data.callsign || "-"}`;
          injectInput.value = "";
        } else {
          const err = await res.json();
          injectFeedback.className = "inject-feedback error";
          injectFeedback.textContent = `Error: ${err.detail || res.statusText}`;
        }
      } catch (e) {
        injectFeedback.className = "inject-feedback error";
        injectFeedback.textContent = `Network error: ${e.message}`;
      }
    });
  }

  // Flight Creation Modal Elements
  const btnNewFlight = document.getElementById("btn-new-flight");
  const flightModal = document.getElementById("flight-modal");
  const btnCloseFlightModal = document.getElementById("btn-close-flight-modal");
  const btnCancelFlight = document.getElementById("btn-cancel-flight");
  const createFlightForm = document.getElementById("create-flight-form");
  const selectOrigin = document.getElementById("select-origin");
  const selectDestination = document.getElementById("select-destination");
  const selectAircraftType = document.getElementById("select-aircraft-type");
  const inputCallsign = document.getElementById("input-callsign");
  const inputCruiseAlt = document.getElementById("input-cruise-alt");
  const inputSquawk = document.getElementById("input-squawk");
  const flightModalFeedback = document.getElementById("flight-modal-feedback");

  // Load Airports & Aircraft Types into Dropdowns
  async function loadFlightCreationOptions() {
    try {
      const [airportsRes, typesRes] = await Promise.all([
        fetch("/api/v1/airports"),
        fetch("/api/v1/aircraft-types"),
      ]);

      if (airportsRes.ok && selectOrigin && selectDestination) {
        const airports = await airportsRes.json();
        selectOrigin.innerHTML = '<option value="" disabled selected>Select Origin...</option>';
        selectDestination.innerHTML = '<option value="" disabled selected>Select Destination...</option>';

        airports.forEach((apt) => {
          const opt1 = document.createElement("option");
          opt1.value = apt.icao;
          opt1.textContent = `${apt.icao} — ${apt.name} (${apt.city || apt.country})`;
          selectOrigin.appendChild(opt1);

          const opt2 = document.createElement("option");
          opt2.value = apt.icao;
          opt2.textContent = `${apt.icao} — ${apt.name} (${apt.city || apt.country})`;
          selectDestination.appendChild(opt2);
        });

        // Sensible defaults: EHAM -> EGLL
        if (selectOrigin.querySelector('option[value="EHAM"]')) {
          selectOrigin.value = "EHAM";
        }
        if (selectDestination.querySelector('option[value="EGLL"]')) {
          selectDestination.value = "EGLL";
        }
      }

      if (typesRes.ok && selectAircraftType) {
        const types = await typesRes.json();
        selectAircraftType.innerHTML = '<option value="" disabled selected>Select Aircraft...</option>';
        types.forEach((t) => {
          const code = typeof t === "string" ? t : t.type_code;
          const label = typeof t === "string" ? t : `${t.type_code} — ${t.description || ""}`;
          const opt = document.createElement("option");
          opt.value = code;
          opt.textContent = label;
          selectAircraftType.appendChild(opt);
        });
        if (selectAircraftType.querySelector('option[value="B738"]')) {
          selectAircraftType.value = "B738";
        }
      }
    } catch (err) {
      console.warn("Could not load flight creation options:", err);
    }
  }

  loadFlightCreationOptions();

  // Modal Open / Close Handlers
  function openFlightModal() {
    if (!flightModal) return;
    flightModal.classList.remove("hidden");
    if (flightModalFeedback) {
      flightModalFeedback.className = "modal-feedback hidden";
      flightModalFeedback.textContent = "";
    }
  }

  function closeFlightModal() {
    if (!flightModal) return;
    flightModal.classList.add("hidden");
  }

  if (btnNewFlight) {
    btnNewFlight.addEventListener("click", openFlightModal);
  }
  if (btnCloseFlightModal) {
    btnCloseFlightModal.addEventListener("click", closeFlightModal);
  }
  if (btnCancelFlight) {
    btnCancelFlight.addEventListener("click", closeFlightModal);
  }

  if (flightModal) {
    flightModal.addEventListener("click", (e) => {
      if (e.target === flightModal) closeFlightModal();
    });
  }

  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && flightModal && !flightModal.classList.contains("hidden")) {
      closeFlightModal();
    }
  });

  // Flight Creation Form Submission
  if (createFlightForm) {
    createFlightForm.addEventListener("submit", async (e) => {
      e.preventDefault();
      const origin = selectOrigin ? selectOrigin.value : "";
      const destination = selectDestination ? selectDestination.value : "";
      const aircraftType = selectAircraftType ? selectAircraftType.value : "";
      const callsign = inputCallsign ? inputCallsign.value.trim() : "";
      const cruiseAlt = inputCruiseAlt && inputCruiseAlt.value ? parseFloat(inputCruiseAlt.value) : null;
      const squawk = (inputSquawk && inputSquawk.value.trim()) || "1200";

      if (origin === destination) {
        if (flightModalFeedback) {
          flightModalFeedback.className = "modal-feedback error";
          flightModalFeedback.textContent = "Error: Origin and Destination airports must be different.";
          flightModalFeedback.classList.remove("hidden");
        }
        return;
      }

      const payload = {
        origin_icao: origin,
        destination_icao: destination,
        aircraft_type: aircraftType,
        callsign: callsign || undefined,
        cruise_alt_ft: cruiseAlt || undefined,
        squawk: squawk,
      };

      try {
        const res = await fetch("/api/v1/flights", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });

        if (res.status === 201) {
          const data = await res.json();
          if (flightModalFeedback) {
            flightModalFeedback.className = "modal-feedback success";
            flightModalFeedback.textContent = `🚀 Flight launched! ${data.callsign} (${data.icao_hex}) ${data.origin} -> ${data.destination} [${data.distance_nm} NM]`;
            flightModalFeedback.classList.remove("hidden");
          }

          // Auto close after 1.4s and select aircraft on radar/map
          setTimeout(() => {
            closeFlightModal();
            if (data.icao_hex) {
              radar.setSelectedIcao(data.icao_hex);
            }
          }, 1400);
        } else {
          const errData = await res.json();
          if (flightModalFeedback) {
            flightModalFeedback.className = "modal-feedback error";
            flightModalFeedback.textContent = `Error: ${errData.detail || res.statusText}`;
            flightModalFeedback.classList.remove("hidden");
          }
        }
      } catch (networkErr) {
        if (flightModalFeedback) {
          flightModalFeedback.className = "modal-feedback error";
          flightModalFeedback.textContent = `Network error: ${networkErr.message}`;
          flightModalFeedback.classList.remove("hidden");
        }
      }
    });
  }

  // ============================================================================
  // M3: Ground Receiver Placement & Management
  // ============================================================================
  const btnAddReceiver = document.getElementById("btn-add-receiver");
  const receiverModal = document.getElementById("receiver-modal");
  const btnCloseReceiverModal = document.getElementById("btn-close-receiver-modal");
  const btnCancelRx = document.getElementById("btn-cancel-rx");
  const createReceiverForm = document.getElementById("create-receiver-form");
  const btnRxPickMap = document.getElementById("btn-rx-pick-map");
  const rxInputId = document.getElementById("rx-input-id");
  const rxInputName = document.getElementById("rx-input-name");
  const rxInputLat = document.getElementById("rx-input-lat");
  const rxInputLon = document.getElementById("rx-input-lon");
  const rxInputAlt = document.getElementById("rx-input-alt");
  const rxInputGain = document.getElementById("rx-input-gain");
  const rxInputSens = document.getElementById("rx-input-sens");
  const rxModalFeedback = document.getElementById("rx-modal-feedback");

  function openReceiverModal() {
    if (!receiverModal) return;
    receiverModal.classList.remove("hidden");
    if (rxModalFeedback) {
      rxModalFeedback.className = "modal-feedback hidden";
      rxModalFeedback.textContent = "";
    }
  }

  function closeReceiverModal() {
    if (!receiverModal) return;
    receiverModal.classList.add("hidden");
  }

  if (btnAddReceiver) btnAddReceiver.addEventListener("click", openReceiverModal);
  if (btnCloseReceiverModal) btnCloseReceiverModal.addEventListener("click", closeReceiverModal);
  if (btnCancelRx) btnCancelRx.addEventListener("click", closeReceiverModal);

  if (receiverModal) {
    receiverModal.addEventListener("click", (e) => {
      if (e.target === receiverModal) closeReceiverModal();
    });
  }

  if (btnRxPickMap) {
    btnRxPickMap.addEventListener("click", () => {
      closeReceiverModal();
      if (btnToggleMap) btnToggleMap.click();
      map.armCoordinatePicker((lat, lon) => {
        if (rxInputLat) rxInputLat.value = lat.toFixed(4);
        if (rxInputLon) rxInputLon.value = lon.toFixed(4);
        openReceiverModal();
      });
    });
  }

  if (createReceiverForm) {
    createReceiverForm.addEventListener("submit", async (e) => {
      e.preventDefault();
      const payload = {
        rx_id: rxInputId?.value.trim() || undefined,
        name: rxInputName?.value.trim() || undefined,
        lat_deg: parseFloat(rxInputLat.value),
        lon_deg: parseFloat(rxInputLon.value),
        alt_msl_m: parseFloat(rxInputAlt?.value || "10"),
        antenna_gain_peak_dbi: parseFloat(rxInputGain?.value || "5.0"),
        sensitivity_dbm: parseFloat(rxInputSens?.value || "-84.0"),
      };

      try {
        const res = await fetch("/api/v1/receivers", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });

        if (res.status === 201) {
          const rx = await res.json();
          if (rxModalFeedback) {
            rxModalFeedback.className = "modal-feedback success";
            rxModalFeedback.textContent = `🗼 Station placed: ${rx.name} (${rx.rx_id})`;
            rxModalFeedback.classList.remove("hidden");
          }
          setTimeout(() => {
            closeReceiverModal();
            createReceiverForm.reset();
          }, 1200);
        } else {
          const err = await res.json();
          if (rxModalFeedback) {
            rxModalFeedback.className = "modal-feedback error";
            rxModalFeedback.textContent = `Error: ${err.detail || res.statusText}`;
            rxModalFeedback.classList.remove("hidden");
          }
        }
      } catch (err) {
        if (rxModalFeedback) {
          rxModalFeedback.className = "modal-feedback error";
          rxModalFeedback.textContent = `Network error: ${err.message}`;
          rxModalFeedback.classList.remove("hidden");
        }
      }
    });
  }

  // ============================================================================
  // M3: RF Interference / Jammer Deployment & Management
  // ============================================================================
  const btnAddJammer = document.getElementById("btn-add-jammer");
  const jammerModal = document.getElementById("jammer-modal");
  const btnCloseJammerModal = document.getElementById("btn-close-jammer-modal");
  const btnCancelJam = document.getElementById("btn-cancel-jam");
  const createJammerForm = document.getElementById("create-jammer-form");
  const btnJamPickMap = document.getElementById("btn-jam-pick-map");
  const jamInputId = document.getElementById("jam-input-id");
  const jamInputName = document.getElementById("jam-input-name");
  const jamInputLat = document.getElementById("jam-input-lat");
  const jamInputLon = document.getElementById("jam-input-lon");
  const jamInputPower = document.getElementById("jam-input-power");
  const powerValLabel = document.getElementById("power-val-label");
  const jamInputFreq = document.getElementById("jam-input-freq");
  const jamInputBw = document.getElementById("jam-input-bw");
  const jamInputActive = document.getElementById("jam-input-active");
  const jamModalFeedback = document.getElementById("jam-modal-feedback");

  function openJammerModal() {
    if (!jammerModal) return;
    jammerModal.classList.remove("hidden");
    if (jamModalFeedback) {
      jamModalFeedback.className = "modal-feedback hidden";
      jamModalFeedback.textContent = "";
    }
  }

  function closeJammerModal() {
    if (!jammerModal) return;
    jammerModal.classList.add("hidden");
  }

  if (btnAddJammer) btnAddJammer.addEventListener("click", openJammerModal);
  if (btnCloseJammerModal) btnCloseJammerModal.addEventListener("click", closeJammerModal);
  if (btnCancelJam) btnCancelJam.addEventListener("click", closeJammerModal);

  if (jammerModal) {
    jammerModal.addEventListener("click", (e) => {
      if (e.target === jammerModal) closeJammerModal();
    });
  }

  if (jamInputPower && powerValLabel) {
    jamInputPower.addEventListener("input", () => {
      const dbm = parseFloat(jamInputPower.value);
      const watts = Math.pow(10, (dbm - 30) / 10);
      powerValLabel.textContent = `${dbm} dBm (${watts < 1 ? (watts * 1000).toFixed(0) + " mW" : watts.toFixed(1) + " W"})`;
    });
  }

  if (btnJamPickMap) {
    btnJamPickMap.addEventListener("click", () => {
      closeJammerModal();
      if (btnToggleMap) btnToggleMap.click();
      map.armCoordinatePicker((lat, lon) => {
        if (jamInputLat) jamInputLat.value = lat.toFixed(4);
        if (jamInputLon) jamInputLon.value = lon.toFixed(4);
        openJammerModal();
      });
    });
  }

  if (createJammerForm) {
    createJammerForm.addEventListener("submit", async (e) => {
      e.preventDefault();
      const payload = {
        source_id: jamInputId?.value.trim() || undefined,
        name: jamInputName?.value.trim() || undefined,
        lat_deg: parseFloat(jamInputLat.value),
        lon_deg: parseFloat(jamInputLon.value),
        tx_power_dbm: parseFloat(jamInputPower?.value || "40.0"),
        frequency_hz: parseFloat(jamInputFreq?.value || "1090.0") * 1e6,
        bandwidth_mhz: parseFloat(jamInputBw?.value || "2.0"),
        is_active: jamInputActive ? jamInputActive.checked : true,
      };

      try {
        const res = await fetch("/api/v1/interference", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
        });

        if (res.status === 201) {
          const jam = await res.json();
          if (jamModalFeedback) {
            jamModalFeedback.className = "modal-feedback success";
            jamModalFeedback.textContent = `☢ Jammer deployed: ${jam.name} (${jam.tx_power_dbm} dBm)`;
            jamModalFeedback.classList.remove("hidden");
          }
          setTimeout(() => {
            closeJammerModal();
            createJammerForm.reset();
          }, 1200);
        } else {
          const err = await res.json();
          if (jamModalFeedback) {
            jamModalFeedback.className = "modal-feedback error";
            jamModalFeedback.textContent = `Error: ${err.detail || res.statusText}`;
            jamModalFeedback.classList.remove("hidden");
          }
        }
      } catch (err) {
        if (jamModalFeedback) {
          jamModalFeedback.className = "modal-feedback error";
          jamModalFeedback.textContent = `Network error: ${err.message}`;
          jamModalFeedback.classList.remove("hidden");
        }
      }
    });
  }

  // ============================================================================
  // M3: Sensor HUD Drawer & Global Action Handlers
  // ============================================================================
  const btnToggleSensors = document.getElementById("btn-toggle-sensors");
  const sensorHud = document.getElementById("sensor-hud");
  const sensorHudClose = document.getElementById("sensor-hud-close");

  if (btnToggleSensors && sensorHud) {
    btnToggleSensors.addEventListener("click", () => {
      sensorHud.classList.toggle("closed");
    });
  }

  if (sensorHudClose && sensorHud) {
    sensorHudClose.addEventListener("click", () => {
      sensorHud.classList.add("closed");
    });
  }

  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
      if (receiverModal && !receiverModal.classList.contains("hidden")) closeReceiverModal();
      if (jammerModal && !jammerModal.classList.contains("hidden")) closeJammerModal();
      if (sensorHud && !sensorHud.classList.contains("closed")) sensorHud.classList.add("closed");
    }
  });

  window.deleteReceiver = async (rxId) => {
    try {
      const res = await fetch(`/api/v1/receivers/${encodeURIComponent(rxId)}`, { method: "DELETE" });
      if (res.ok) {
        console.log(`Deleted receiver: ${rxId}`);
      }
    } catch (err) {
      console.error("Failed to delete receiver:", err);
    }
  };

  window.deleteJammer = async (sourceId) => {
    try {
      const res = await fetch(`/api/v1/interference/${encodeURIComponent(sourceId)}`, { method: "DELETE" });
      if (res.ok) {
        console.log(`Deleted jammer: ${sourceId}`);
      }
    } catch (err) {
      console.error("Failed to delete jammer:", err);
    }
  };

  window.toggleJammer = async (sourceId) => {
    try {
      const res = await fetch(`/api/v1/interference/${encodeURIComponent(sourceId)}/toggle`, { method: "POST" });
      if (res.ok) {
        console.log(`Toggled jammer: ${sourceId}`);
      }
    } catch (err) {
      console.error("Failed to toggle jammer:", err);
    }
  };

  function updateSpectrumHealth(receivers, jammers) {
    const healthEl = document.getElementById("kpi-health-status");
    if (!healthEl) return;
    const jammedCount = (receivers || []).filter((r) => r.status === "JAMMED").length;
    const degradedCount = (receivers || []).filter((r) => r.status === "DEGRADED").length;
    if (jammedCount > 0) {
      healthEl.className = "kpi-val status-pill jammed";
      healthEl.textContent = "JAMMED";
    } else if (degradedCount > 0) {
      healthEl.className = "kpi-val status-pill degraded";
      healthEl.textContent = "DEGRADED";
    } else {
      healthEl.className = "kpi-val status-pill normal";
      healthEl.textContent = "OPTIMAL";
    }
  }

  function updateSensorHudReceivers(receivers) {
    const container = document.getElementById("sensor-rx-list");
    if (!container) return;
    container.innerHTML = "";
    (receivers || []).forEach((rx) => {
      const card = document.createElement("div");
      const status = rx.status || "NORMAL";
      card.className = `sensor-card ${status.toLowerCase()}`;
      const sinr = rx.metrics ? rx.metrics.sinr_db.toFixed(1) : "--";
      const loss = rx.metrics ? (rx.metrics.packet_loss_rate * 100).toFixed(1) : "0.0";
      const noise = rx.metrics ? rx.metrics.noise_floor_eff_dbm.toFixed(1) : (rx.noise_floor_dbm ?? -90.0);
      card.innerHTML = `
        <div class="card-header">
          <span class="card-title">🗼 ${rx.name || rx.rx_id}</span>
          <div class="card-actions">
            <span class="status-pill ${status.toLowerCase()}">${status}</span>
            <button class="card-del-btn" onclick="window.deleteReceiver('${rx.rx_id}')" title="Delete receiver">✕</button>
          </div>
        </div>
        <div style="display:flex;justify-content:space-between;color:#94a3b8;font-size:10px;">
          <span>SINR: <b style="color:#f8fafc;">${sinr} dB</b></span>
          <span>NOISE: <b style="color:#f8fafc;">${noise} dBm</b></span>
          <span>LOSS: <b style="color:${parseFloat(loss) > 5 ? '#ef4444' : '#00ff99'};">${loss}%</b></span>
        </div>
      `;
      container.appendChild(card);
    });
  }

  function updateSensorHudJammers(jammers) {
    const container = document.getElementById("sensor-jammer-list");
    if (!container) return;
    container.innerHTML = "";
    (jammers || []).forEach((jam) => {
      const card = document.createElement("div");
      card.className = "sensor-card jammed";
      card.style.borderLeftColor = jam.is_active ? "#ef4444" : "#64748b";
      card.innerHTML = `
        <div class="card-header">
          <span class="card-title" style="color:${jam.is_active ? "#ef4444" : "#64748b"};">☢ ${jam.name || jam.source_id}</span>
          <div class="card-actions">
            <button class="card-toggle-btn" onclick="window.toggleJammer('${jam.source_id}')">${jam.is_active ? "MUTE" : "UNMUTE"}</button>
            <button class="card-del-btn" onclick="window.deleteJammer('${jam.source_id}')" title="Delete jammer">✕</button>
          </div>
        </div>
        <div style="display:flex;justify-content:space-between;color:#94a3b8;font-size:10px;">
          <span>PWR: <b style="color:#f8fafc;">${jam.tx_power_dbm} dBm (${jam.tx_power_w} W)</b></span>
          <span>STATUS: <b style="color:${jam.is_active ? "#ef4444" : "#64748b"};">${jam.is_active ? "ACTIVE" : "STANDBY"}</b></span>
        </div>
      `;
      container.appendChild(card);
    });
  }

  // Initial load of receivers and jammers via REST API
  async function loadInitialSensorNetwork() {
    try {
      const [rxRes, jamRes] = await Promise.all([
        fetch("/api/v1/receivers"),
        fetch("/api/v1/interference"),
      ]);
      if (rxRes.ok) {
        const receivers = await rxRes.json();
        map.updateReceivers(receivers);
        updateSensorHudReceivers(receivers);
        const rxCountEl = document.getElementById("kpi-rx-count");
        if (rxCountEl) rxCountEl.textContent = receivers.length;
        const sensorCountEl = document.getElementById("sensor-count");
        if (sensorCountEl) sensorCountEl.textContent = receivers.length;
      }
      if (jamRes.ok) {
        const jammers = await jamRes.json();
        map.updateInterference(jammers);
        updateSensorHudJammers(jammers);
        const jamCountEl = document.getElementById("kpi-jammer-count");
        if (jamCountEl) {
          const activeCount = jammers.filter((s) => s.is_active).length;
          jamCountEl.textContent = activeCount;
        }
      }
    } catch (e) {
      console.warn("Could not load initial sensor network:", e);
    }
  }

  loadInitialSensorNetwork();
});
