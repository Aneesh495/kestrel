/**
 * Kestrel Ground Control Station & Avionics HUD Engine.
 * Implements high-frame-rate HTML5 Canvas Primary Flight Display (PFD)
 * and 2D/3D Tactical Mission Radar.
 */

class AvionicsHUD {
    constructor() {
        this.pfdCanvas = document.getElementById("pfdCanvas");
        this.pfdCtx = this.pfdCanvas.getContext("2d");

        this.radarCanvas = document.getElementById("tacticalCanvas");
        this.radarCtx = this.radarCanvas.getContext("2d");

        // Flight telemetry state
        this.state = {
            time: 0.0,
            roll: 0.05,        // rad
            pitch: -0.12,      // rad
            yaw: 0.59,         // rad (34 deg)
            altitude: 15.2,    // meters MSL
            vsi: 0.0,          // m/s vertical speed
            ias: 12.5,         // m/s indicated airspeed
            battery: 94.8,
            posX: 25.4,
            posY: 18.2,
            posZ: -15.2,
            motors: [0.55, 0.58, 0.56, 0.57]
        };

        this.trajectoryHistory = [];
        this.is3D = false;

        this.setupEventListeners();
        this.startLoop();
    }

    setupEventListeners() {
        document.getElementById("btn2D").addEventListener("click", (e) => {
            this.is3D = false;
            e.target.classList.add("active");
            document.getElementById("btn3D").classList.remove("active");
        });

        document.getElementById("btn3D").addEventListener("click", (e) => {
            this.is3D = true;
            e.target.classList.add("active");
            document.getElementById("btn2D").classList.remove("active");
        });

        document.getElementById("btnTakeoff").addEventListener("click", () => this.logAction("Command: AUTO TAKEOFF sent"));
        document.getElementById("btnLoiter").addEventListener("click", () => this.logAction("Command: LOITER HOLD engaged"));
        document.getElementById("btnRTH").addEventListener("click", () => this.logAction("Command: RETURN TO HOME engaged"));
        document.getElementById("btnLand").addEventListener("click", () => this.logAction("Command: PRECISION LANDING engaged"));
        document.getElementById("btnCutoff").addEventListener("click", () => this.logAction("CRITICAL: EMERGENCY MOTOR CUTOFF SENT"));
    }

    logAction(text) {
        const box = document.getElementById("logBox");
        const entry = document.createElement("div");
        entry.className = "log-entry active";
        const tStr = this.state.time.toFixed(2);
        entry.textContent = `[${tStr}s] ${text}`;
        box.prepend(entry);
    }

    startLoop() {
        const tick = () => {
            this.updateSimState();
            this.renderPFD();
            this.renderRadar();
            this.updateUI();
            requestAnimationFrame(tick);
        };
        requestAnimationFrame(tick);
    }

    updateSimState() {
        this.state.time += 0.016;
        // Mild simulated aerodynamic disturbances
        this.state.roll = 0.04 * Math.sin(this.state.time * 0.8) + 0.02 * Math.sin(this.state.time * 2.5);
        this.state.pitch = -0.10 + 0.03 * Math.cos(this.state.time * 0.6);
        this.state.yaw += 0.002;
        if (this.state.yaw > Math.PI * 2) this.state.yaw -= Math.PI * 2;

        this.state.posX += Math.cos(this.state.yaw) * 0.1;
        this.state.posY += Math.sin(this.state.yaw) * 0.1;

        if (this.trajectoryHistory.length > 300) this.trajectoryHistory.shift();
        this.trajectoryHistory.push({ x: this.state.posX, y: this.state.posY, z: this.state.posZ });
    }

    renderPFD() {
        const ctx = this.pfdCtx;
        const w = this.pfdCanvas.width;
        const h = this.pfdCanvas.height;
        const cx = w / 2;
        const cy = h / 2;

        ctx.clearRect(0, 0, w, h);

        // 1. Artificial Horizon Ball
        ctx.save();
        ctx.beginPath();
        ctx.arc(cx, cy, 180, 0, Math.PI * 2);
        ctx.clip();

        ctx.translate(cx, cy);
        ctx.rotate(-this.state.roll);
        const pitchOffset = this.state.pitch * 300; // pixels per rad

        // Sky
        ctx.fillStyle = "#103454";
        ctx.fillRect(-250, -350 + pitchOffset, 500, 350);

        // Ground
        ctx.fillStyle = "#382312";
        ctx.fillRect(-250, 0 + pitchOffset, 500, 350);

        // Horizon line
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-250, pitchOffset);
        ctx.lineTo(250, pitchOffset);
        ctx.stroke();

        // Pitch Ladder
        ctx.strokeStyle = "rgba(255, 255, 255, 0.85)";
        ctx.fillStyle = "#ffffff";
        ctx.font = "10px monospace";
        ctx.textAlign = "center";

        for (let deg = -60; deg <= 60; deg += 10) {
            if (deg === 0) continue;
            const y = pitchOffset - (deg * (Math.PI / 180)) * 300;
            const lineLen = deg % 20 === 0 ? 40 : 24;

            ctx.beginPath();
            ctx.moveTo(-lineLen, y);
            ctx.lineTo(lineLen, y);
            ctx.stroke();

            if (deg % 20 === 0) {
                ctx.fillText(deg.toString(), -lineLen - 14, y + 4);
                ctx.fillText(deg.toString(), lineLen + 14, y + 4);
            }
        }
        ctx.restore();

        // 2. Roll Angle Arc
        ctx.save();
        ctx.strokeStyle = "rgba(0, 243, 255, 0.6)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(cx, cy, 185, Math.PI + 0.5, Math.PI * 2 - 0.5);
        ctx.stroke();

        // Roll pointer
        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(-this.state.roll);
        ctx.fillStyle = "#00f3ff";
        ctx.beginPath();
        ctx.moveTo(0, -185);
        ctx.lineTo(-6, -195);
        ctx.lineTo(6, -195);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
        ctx.restore();

        // 3. Central Flight Director Reticle
        ctx.strokeStyle = "#00ff88";
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        // Left wing
        ctx.moveTo(cx - 50, cy);
        ctx.lineTo(cx - 15, cy);
        ctx.lineTo(cx - 15, cy + 8);
        // Center pip
        ctx.arc(cx, cy, 3, 0, Math.PI * 2);
        // Right wing
        ctx.moveTo(cx + 15, cy + 8);
        ctx.lineTo(cx + 15, cy);
        ctx.lineTo(cx + 50, cy);
        ctx.stroke();

        // 4. Airspeed Tape (Left)
        ctx.fillStyle = "rgba(8, 14, 22, 0.85)";
        ctx.fillRect(10, 40, 50, 380);
        ctx.strokeStyle = "#1a2538";
        ctx.strokeRect(10, 40, 50, 380);

        ctx.fillStyle = "#00f3ff";
        ctx.font = "11px monospace";
        ctx.textAlign = "right";
        for (let spd = 0; spd <= 25; spd += 5) {
            const y = cy - (spd - this.state.ias) * 14;
            if (y > 45 && y < 415) {
                ctx.fillText(spd.toString(), 48, y + 4);
                ctx.beginPath();
                ctx.moveTo(52, y);
                ctx.lineTo(58, y);
                ctx.stroke();
            }
        }

        // 5. Altitude Tape (Right)
        ctx.fillStyle = "rgba(8, 14, 22, 0.85)";
        ctx.fillRect(w - 60, 40, 50, 380);
        ctx.strokeRect(w - 60, 40, 50, 380);

        ctx.fillStyle = "#00f3ff";
        ctx.textAlign = "left";
        for (let alt = 0; alt <= 40; alt += 5) {
            const y = cy - (alt - this.state.altitude) * 14;
            if (y > 45 && y < 415) {
                ctx.fillText(alt.toString(), w - 46, y + 4);
                ctx.beginPath();
                ctx.moveTo(w - 58, y);
                ctx.lineTo(w - 52, y);
                ctx.stroke();
            }
        }
    }

    renderRadar() {
        const ctx = this.radarCtx;
        const w = this.radarCanvas.width;
        const h = this.radarCanvas.height;
        const cx = w / 2;
        const cy = h / 2;

        ctx.clearRect(0, 0, w, h);

        // Grid rings
        ctx.strokeStyle = "rgba(26, 37, 56, 0.6)";
        ctx.lineWidth = 1;
        for (let r = 50; r <= 220; r += 50) {
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Crosshairs
        ctx.beginPath();
        ctx.moveTo(cx, 10);
        ctx.lineTo(cx, h - 10);
        ctx.moveTo(10, cy);
        ctx.lineTo(w - 10, cy);
        ctx.stroke();

        // Waypoints
        const scale = 5.0; // pixels per meter
        const wps = [
            { x: 0, y: 0, label: "HOME" },
            { x: 25, y: 0, label: "WP1" },
            { x: 25, y: 25, label: "WP2" },
            { x: 0, y: 25, label: "WP3" }
        ];

        ctx.strokeStyle = "rgba(0, 243, 255, 0.4)";
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        wps.forEach((wp, idx) => {
            const px = cx + wp.x * scale;
            const py = cy - wp.y * scale;
            if (idx === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.stroke();
        ctx.setLineDash([]);

        // Waypoint markers
        ctx.fillStyle = "#ffb800";
        ctx.font = "9px monospace";
        ctx.textAlign = "center";
        wps.forEach(wp => {
            const px = cx + wp.x * scale;
            const py = cy - wp.y * scale;
            ctx.beginPath();
            ctx.arc(px, py, 4, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillText(wp.label, px, py - 8);
        });

        // Flight trail
        if (this.trajectoryHistory.length > 1) {
            ctx.strokeStyle = "rgba(0, 255, 136, 0.5)";
            ctx.lineWidth = 2;
            ctx.beginPath();
            this.trajectoryHistory.forEach((pt, idx) => {
                const px = cx + pt.x * scale;
                const py = cy - pt.y * scale;
                if (idx === 0) ctx.moveTo(px, py);
                else ctx.lineTo(px, py);
            });
            ctx.stroke();
        }

        // Active vehicle icon
        const vx = cx + this.state.posX * scale;
        const vy = cy - this.state.posY * scale;
        ctx.save();
        ctx.translate(vx, vy);
        ctx.rotate(this.state.yaw);

        ctx.fillStyle = "#00f3ff";
        ctx.beginPath();
        ctx.moveTo(0, -10);
        ctx.lineTo(-7, 8);
        ctx.lineTo(0, 4);
        ctx.lineTo(7, 8);
        ctx.closePath();
        ctx.fill();

        ctx.restore();
    }

    updateUI() {
        document.getElementById("valIas").textContent = this.state.ias.toFixed(1) + " m/s";
        document.getElementById("valAlt").textContent = this.state.altitude.toFixed(1) + " m";
        document.getElementById("valVsi").textContent = (this.state.vsi >= 0 ? "+" : "") + this.state.vsi.toFixed(1) + " m/s";
        const deg = (this.state.yaw * 180 / Math.PI).toFixed(0).padStart(3, '0');
        document.getElementById("valHdg").textContent = deg + " deg";

        document.getElementById("battPercent").textContent = this.state.battery.toFixed(1) + "%";
        document.getElementById("m1Fill").style.width = (this.state.motors[0] * 100) + "%";
        document.getElementById("m1Val").textContent = (this.state.motors[0] * 100).toFixed(0) + "%";
        document.getElementById("m2Fill").style.width = (this.state.motors[1] * 100) + "%";
        document.getElementById("m2Val").textContent = (this.state.motors[1] * 100).toFixed(0) + "%";
        document.getElementById("m3Fill").style.width = (this.state.motors[2] * 100) + "%";
        document.getElementById("m3Val").textContent = (this.state.motors[2] * 100).toFixed(0) + "%";
        document.getElementById("m4Fill").style.width = (this.state.motors[3] * 100) + "%";
        document.getElementById("m4Val").textContent = (this.state.motors[3] * 100).toFixed(0) + "%";
    }
}

window.addEventListener("DOMContentLoaded", () => {
    new AvionicsHUD();
});
