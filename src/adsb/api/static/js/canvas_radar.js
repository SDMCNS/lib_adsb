/**
 * HTML5 Canvas ATC Radar Scope Renderer.
 *
 * Implements 20 NM range rings, 30° bearing radials, continuous CRT sweep,
 * 5-band altitude coloring, 1-min velocity leader vectors, 3-line ATC data blocks,
 * and mouse hit-testing.
 */

export class CanvasRadar {
  constructor(canvasId, { onSelectTrack } = {}) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.onSelectTrack = onSelectTrack || (() => {});

    // Radar Center Reference (Schiphol EHAM by default)
    this.centerLat = 52.3105;
    this.centerLon = 4.7683;
    this.maxRangeNm = 25.0; // Viewport radius in NM

    this.tracks = [];
    this.selectedIcao = null;

    this.sweepAngleRad = 0.0;
    this.sweepSpeedRadPerSec = 0.8; // ~8 rpm rotation
    this.lastFrameTime = performance.now();

    this._setupCanvas();
    this._bindEvents();
    this._startRenderLoop();
  }

  _setupCanvas() {
    this._resize();
    window.addEventListener("resize", () => this._resize());
  }

  resize() {
    this._resize();
  }

  _resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.scale(dpr, dpr);
    this.width = rect.width;
    this.height = rect.height;
  }

  _bindEvents() {
    this.canvas.addEventListener("click", (evt) => {
      const rect = this.canvas.getBoundingClientRect();
      const clickX = evt.clientX - rect.left;
      const clickY = evt.clientY - rect.top;
      this._handleCanvasClick(clickX, clickY);
    });
  }

  updateTracks(tracks) {
    this.tracks = tracks || [];
  }

  setSelectedIcao(icaoHex) {
    this.selectedIcao = icaoHex;
  }

  _getAltitudeColor(altFt) {
    if (altFt === null || altFt === undefined) return "#94a3b8"; // Muted gray
    if (altFt < 5000) return "#00ff99"; // Green
    if (altFt < 15000) return "#00e5ff"; // Cyan
    if (altFt < 25000) return "#3b82f6"; // Blue
    if (altFt < 35000) return "#f59e0b"; // Amber
    return "#ec4899"; // Magenta
  }

  _latLonToCanvas(lat, lon) {
    const latRad = (this.centerLat * Math.PI) / 180.0;
    const dxNm = (lon - this.centerLon) * Math.cos(latRad) * 60.0;
    const dyNm = (lat - this.centerLat) * 60.0;

    const minDim = Math.min(this.width, this.height);
    const pxPerNm = (minDim * 0.45) / this.maxRangeNm;

    const cx = this.width / 2.0;
    const cy = this.height / 2.0;

    return {
      x: cx + dxNm * pxPerNm,
      y: cy - dyNm * pxPerNm,
      pxPerNm,
    };
  }

  _startRenderLoop() {
    const render = (now) => {
      const dt = (now - this.lastFrameTime) / 1000.0;
      this.lastFrameTime = now;

      this.sweepAngleRad = (this.sweepAngleRad + this.sweepSpeedRadPerSec * dt) % (Math.PI * 2);
      this._draw();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  _draw() {
    const ctx = this.ctx;
    const cx = this.width / 2.0;
    const cy = this.height / 2.0;
    const minDim = Math.min(this.width, this.height);
    const radius = minDim * 0.45;

    // Clear background
    ctx.fillStyle = "#0a0e17";
    ctx.fillRect(0, 0, this.width, this.height);

    // 1. Draw Range Rings (5, 10, 15, 20, 25 NM)
    const pxPerNm = radius / this.maxRangeNm;
    const rings = [5, 10, 15, 20, 25];

    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(30, 41, 59, 0.8)";
    ctx.font = "10px SF Mono, Consolas, monospace";
    ctx.fillStyle = "rgba(100, 116, 139, 0.7)";

    rings.forEach((nm) => {
      const r = nm * pxPerNm;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.stroke();

      // Range text labels
      ctx.fillText(`${nm} NM`, cx + 4, cy - r + 12);
    });

    // 2. Draw Bearing Radials (every 30 degrees)
    for (let deg = 0; deg < 360; deg += 30) {
      const rad = ((deg - 90) * Math.PI) / 180.0;
      const x2 = cx + radius * Math.cos(rad);
      const y2 = cy + radius * Math.sin(rad);

      ctx.strokeStyle = "rgba(30, 41, 59, 0.5)";
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(x2, y2);
      ctx.stroke();

      // Degree markings at ring perimeter
      const labelX = cx + (radius + 14) * Math.cos(rad);
      const labelY = cy + (radius + 14) * Math.sin(rad);
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(`${deg.toString().padStart(3, "0")}°`, labelX, labelY);
    }

    // 3. Draw Radar Phosphor Sweep Line
    const sweepX = cx + radius * Math.cos(this.sweepAngleRad);
    const sweepY = cy + radius * Math.sin(this.sweepAngleRad);

    const sweepGrad = ctx.createRadialGradient(cx, cy, 0, cx, cy, radius);
    sweepGrad.addColorStop(0, "rgba(0, 255, 153, 0.4)");
    sweepGrad.addColorStop(1, "rgba(0, 255, 153, 0.05)");

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, radius, this.sweepAngleRad - 0.25, this.sweepAngleRad);
    ctx.closePath();
    ctx.fillStyle = sweepGrad;
    ctx.fill();

    ctx.strokeStyle = "rgba(0, 255, 153, 0.7)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(sweepX, sweepY);
    ctx.stroke();

    // Center Aerodrome Marker
    ctx.fillStyle = "#00ff99";
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fill();

    // 4. Draw Aircraft Targets
    this.tracks.forEach((track) => {
      const lat = track.lat_deg ?? track.lat;
      const lon = track.lon_deg ?? track.lon;
      if (lat === null || lat === undefined || lon === null || lon === undefined) return;

      const pos = this._latLonToCanvas(lat, lon);
      const alt = track.alt_baro_ft ?? track.alt_geom_ft;
      const color = this._getAltitudeColor(alt);
      const isSelected =
        this.selectedIcao &&
        (track.icao_hex === this.selectedIcao ||
          track.icao24?.toString(16).toUpperCase() === this.selectedIcao);

      // Save coords on track object for hit-testing
      track._canvasX = pos.x;
      track._canvasY = pos.y;

      ctx.save();
      ctx.translate(pos.x, pos.y);

      // Target Symbol (Aircraft Chevron / Box)
      const hdgDeg = track.track_deg ?? 0;
      const hdgRad = ((hdgDeg - 90) * Math.PI) / 180.0;

      // Draw Heading Vector
      ctx.save();
      ctx.rotate(hdgRad);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(8, 0);
      ctx.lineTo(-6, -5);
      ctx.lineTo(-3, 0);
      ctx.lineTo(-6, 5);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      // Selected ring highlight
      if (isSelected) {
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(0, 0, 12, 0, Math.PI * 2);
        ctx.stroke();
      }

      // Velocity Leader Vector (1 minute projection line)
      const gsKt = track.groundspeed_kt || 0;
      if (gsKt > 10 && track.track_deg !== null && track.track_deg !== undefined) {
        // Distance traveled in 1 minute = gsKt / 60 NM
        const leaderNm = gsKt / 60.0;
        const leaderPx = leaderNm * pos.pxPerNm;
        const leaderRad = ((track.track_deg - 90) * Math.PI) / 180.0;
        const lx = leaderPx * Math.cos(leaderRad);
        const ly = leaderPx * Math.sin(leaderRad);

        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(lx, ly);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 3-Line ATC Data Block
      const callsign = (track.callsign ? track.callsign.trim() : null) || track.icao_hex || "UNK";
      let altStr = "---";
      if (alt !== null && alt !== undefined) {
        if (alt >= 18000) {
          altStr = `FL${Math.round(alt / 100)}`;
        } else {
          altStr = `A${Math.round(alt / 100).toString().padStart(3, "0")}`;
        }
      }
      const gsStr = gsKt ? `${Math.round(gsKt)}KT` : "---KT";

      ctx.fillStyle = color;
      ctx.font = "bold 10px SF Mono, Consolas, monospace";
      ctx.textAlign = "left";
      ctx.textBaseline = "top";

      const blockX = 14;
      const blockY = -16;

      // Connecting leader to data block
      ctx.strokeStyle = "rgba(100, 116, 139, 0.6)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(blockX, blockY + 12);
      ctx.stroke();

      ctx.fillText(callsign, blockX + 2, blockY);
      ctx.font = "9px SF Mono, Consolas, monospace";
      ctx.fillText(`${altStr}  ${gsStr}`, blockX + 2, blockY + 11);

      // Track State / Anomaly indicator
      if (track.anomalies && track.anomalies.length > 0) {
        ctx.fillStyle = "#ef4444";
        ctx.fillText(`! ${track.anomalies[0]}`, blockX + 2, blockY + 22);
      } else if (track.state === "COASTING") {
        ctx.fillStyle = "#f59e0b";
        ctx.fillText("COAST", blockX + 2, blockY + 22);
      }

      ctx.restore();
    });
  }

  _handleCanvasClick(x, y) {
    const hitRadius = 24;
    for (const track of this.tracks) {
      if (track._canvasX !== undefined && track._canvasY !== undefined) {
        const dx = x - track._canvasX;
        const dy = y - track._canvasY;
        if (Math.hypot(dx, dy) <= hitRadius) {
          this.selectedIcao = track.icao_hex || track.icao24?.toString(16).toUpperCase();
          this.onSelectTrack(track);
          return;
        }
      }
    }
  }
}
