/**
 * ESP32 Radar Viewer — Canvas-based polar visualization, no external dependencies
 */

const CONFIG = {
    MAX_DISTANCE_MM: 7000,
    DANGER_DISTANCE_MM: 1000,
    SAFE_DISTANCE_MM: 6000,
    GRID_STEPS: [1000, 2000, 3000, 4000, 5000, 6000],
    C: {
        bg: '#0b0c10',
        grid: '#1a1a2e',
        gridMajor: '#2a2a4e',
        text: '#888',
        accent: '#00d4ff',
        dangerFill: 'rgba(255,68,68,0.06)',
        safeFill: 'rgba(68,255,68,0.03)',
        dangerLine: 'rgba(255,68,68,0.30)',
        safeLine: 'rgba(68,255,68,0.30)',
        pointDanger: '#ff4444',
        pointSafe: '#44ff44',
        pointNoise: '#00d4ff',
        scan: 'rgba(0,212,255,0.12)'
    }
};

let radarData = [];
let temperature = NaN;
let dangerCount = 0, safeCount = 0, totalCount = 0;
let ws = null, isConnected = false;
let scanAngle = 0;
let canvas, ctx, w, h, cx, cy, r;

function hexToRgba(hex, a) {
    const i = parseInt(hex.slice(1), 16);
    return `rgba(${(i>>16)&0xff},${(i>>8)&0xff},${i&0xff},${a})`;
}

function getColor(d) {
    if (d <= CONFIG.DANGER_DISTANCE_MM) return CONFIG.C.pointDanger;
    if (d <= CONFIG.SAFE_DISTANCE_MM)    return CONFIG.C.pointSafe;
    return CONFIG.C.pointNoise;
}

// angle 0 = north, clockwise. canvas: x→right, y→down
function toCanvas(deg, dist) {
    const rad = deg * Math.PI / 180;
    const s = dist / CONFIG.MAX_DISTANCE_MM * r;
    return { x: cx + s * Math.sin(rad), y: cy - s * Math.cos(rad) };
}

function init() {
    canvas = document.getElementById('radar-canvas');
    ctx = canvas.getContext('2d');
    resize();
    window.addEventListener('resize', resize);
    requestAnimationFrame(loop);
}

function resize() {
    const dpr = devicePixelRatio || 1;
    const R = canvas.parentElement.getBoundingClientRect();
    w = R.width; h = R.height;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    canvas.style.width = w + 'px';
    canvas.style.height = h + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    cx = w / 2; cy = h / 2;
    r = Math.min(cx, cy) * 0.82;
}

function drawZoneFill(maxMM, color) {
    const rr = maxMM / CONFIG.MAX_DISTANCE_MM * r;
    ctx.beginPath();
    ctx.arc(cx, cy, rr, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
}

function drawZoneRing(innerMM, outerMM, color) {
    const ri = innerMM / CONFIG.MAX_DISTANCE_MM * r;
    const ro = outerMM / CONFIG.MAX_DISTANCE_MM * r;
    ctx.beginPath();
    ctx.arc(cx, cy, ro, 0, 2 * Math.PI);
    ctx.arc(cx, cy, ri, 0, 2 * Math.PI, true);
    ctx.fillStyle = color;
    ctx.fill();
}

function drawGrid() {
    CONFIG.GRID_STEPS.forEach(d => {
        const rr = d / CONFIG.MAX_DISTANCE_MM * r;
        ctx.beginPath();
        ctx.arc(cx, cy, rr, 0, 2 * Math.PI);
        const c = d <= CONFIG.DANGER_DISTANCE_MM ? CONFIG.C.dangerLine
                : d <= CONFIG.SAFE_DISTANCE_MM   ? CONFIG.C.safeLine
                : CONFIG.C.grid;
        ctx.strokeStyle = c;
        ctx.lineWidth = 0.5;
        if (d === CONFIG.DANGER_DISTANCE_MM || d === CONFIG.SAFE_DISTANCE_MM) {
            ctx.setLineDash([5, 5]);
            ctx.lineWidth = 1;
        } else { ctx.setLineDash([]); }
        ctx.stroke();
    });
    ctx.setLineDash([]);

    ctx.fillStyle = CONFIG.C.text;
    ctx.font = '9px monospace';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    CONFIG.GRID_STEPS.forEach(d => {
        const rr = d / CONFIG.MAX_DISTANCE_MM * r;
        ctx.fillText((d / 1000) + 'm', cx + rr + 3, cy + 3);
    });

    for (let a = 0; a < 360; a += 30) {
        const rad = a * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + r * Math.sin(rad), cy - r * Math.cos(rad));
        ctx.strokeStyle = CONFIG.C.grid;
        ctx.lineWidth = 0.3;
        ctx.stroke();
    }

    ctx.fillStyle = CONFIG.C.text;
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let a = 0; a < 360; a += 30) {
        const rad = a * Math.PI / 180, lr = r + 16;
        ctx.fillText(a + '°', cx + lr * Math.sin(rad), cy - lr * Math.cos(rad));
    }

    const top = cy - r;
    ctx.beginPath();
    ctx.moveTo(cx, top);
    ctx.lineTo(cx - 5, top + 10);
    ctx.lineTo(cx + 5, top + 10);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0,200,255,0.5)';
    ctx.fill();
}

function drawScan() {
    const rad = scanAngle * Math.PI / 180;
    const dx = Math.sin(rad), dy = -Math.cos(rad);
    const ex = cx + r * dx, ey = cy + r * dy;
    const g = ctx.createLinearGradient(cx, cy, ex, ey);
    g.addColorStop(0, 'transparent');
    g.addColorStop(0.7, 'rgba(0,200,255,0.06)');
    g.addColorStop(1, 'rgba(0,200,255,0.18)');
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = g;
    ctx.lineWidth = 1.5;
    ctx.stroke();
}

function drawPoints() {
    radarData.forEach(p => {
        const pt = toCanvas(p.a, p.d), color = getColor(p.d);
        ctx.save();
        ctx.shadowBlur = 8;
        ctx.shadowColor = hexToRgba(color, 0.5);
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 3.5, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.restore();
    });
}

function loop() {
    ctx.clearRect(0, 0, w, h);
    scanAngle = (scanAngle + 1.5) % 360;
    drawZoneFill(CONFIG.DANGER_DISTANCE_MM, CONFIG.C.dangerFill);
    drawZoneRing(CONFIG.DANGER_DISTANCE_MM, CONFIG.SAFE_DISTANCE_MM, CONFIG.C.safeFill);
    drawGrid();
    drawScan();
    drawPoints();
    requestAnimationFrame(loop);
}

const els = {
    ip: document.getElementById('ipInput'),
    port: document.getElementById('portInput'),
    btn: document.getElementById('connectBtn'),
    status: document.querySelector('.status'),
    statusText: document.getElementById('statusText'),
    temp: document.getElementById('tempDisplay'),
    statDanger: document.getElementById('statDanger'),
    statSafe: document.getElementById('statSafe'),
    statTotal: document.getElementById('statTotal')
};

function toggle() {
    if (ws && ws.readyState === WebSocket.OPEN) disconnect();
    else connect();
}

function connect() {
    const url = `ws://${els.ip.value.trim()}:${els.port.value.trim()}/ws`;
    setState('connecting');
    try {
        ws = new WebSocket(url);
        ws.onopen = () => { isConnected = true; setState('connected'); };
        ws.onmessage = e => { try { handle(JSON.parse(e.data)); } catch (_) {} };
        ws.onerror = () => setState('error');
        ws.onclose = () => { isConnected = false; setState('disconnected'); };
    } catch (_) { alert('Invalid URL'); }
}

function disconnect() {
    if (ws) { ws.close(); ws = null; }
    isConnected = false;
    setState('disconnected');
}

function setState(s) {
    els.status.className = 'status ' + s;
    switch (s) {
        case 'connected':
            els.btn.textContent = '断开'; els.btn.className = 'btn-disconnect';
            els.statusText.textContent = '已连接'; break;
        case 'connecting':
            els.btn.textContent = '...'; els.btn.disabled = true;
            els.statusText.textContent = '连接中...'; break;
        default:
            els.btn.textContent = '连接'; els.btn.className = 'btn-connect';
            els.btn.disabled = false;
            els.statusText.textContent = s === 'error' ? '连接错误' : '未连接';
    }
}

function handle(data) {
    if (data.temp != null && !isNaN(data.temp)) {
        temperature = data.temp;
        els.temp.textContent = data.temp.toFixed(1) + ' °C';
    }
    const vectors = (data.vectors || []).filter(
        v => v.d > 0 && !isNaN(v.d) && v.d <= CONFIG.MAX_DISTANCE_MM
    );
    radarData = vectors;
    dangerCount = vectors.filter(v => v.d <= CONFIG.DANGER_DISTANCE_MM).length;
    safeCount   = vectors.filter(v => v.d > CONFIG.DANGER_DISTANCE_MM && v.d <= CONFIG.SAFE_DISTANCE_MM).length;
    totalCount  = vectors.length;
    els.statDanger.textContent = dangerCount;
    els.statSafe.textContent   = safeCount;
    els.statTotal.textContent  = totalCount;
}

window.onload = function() {
    init();
    els.btn.addEventListener('click', toggle);
    [els.ip, els.port].forEach(el =>
        el.addEventListener('keypress', e => { if (e.key === 'Enter') toggle(); })
    );
};
