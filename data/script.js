const CONFIG = {
    MAX_DISTANCE_MM: 7000,
    DANGER_DISTANCE_MM: 1000,
    SAFE_DISTANCE_MM: 6000,
    GRID_STEPS: [1000, 2000, 3000, 4000, 5000, 6000],
    C: {
        grid: '#1a1a2e',
        text: '#888',
        dangerFill: 'rgba(255,68,68,0.06)',
        safeFill: 'rgba(68,255,68,0.03)',
        dangerLine: 'rgba(255,68,68,0.30)',
        safeLine: 'rgba(68,255,68,0.30)',
        pointDanger: '#ff4444',
        pointSafe: '#44ff44',
        pointNoise: '#00d4ff'
    }
};

const D2R = Math.PI / 180;

let radarData = [];
let cartCmd = null;
let ws = null, isConnected = false;
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

function toCanvas(deg, dist) {
    const rad = deg * D2R;
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
    ctx.fillStyle = CONFIG.C.text;
    ctx.font = '9px monospace';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    CONFIG.GRID_STEPS.forEach(d => {
        const rr = d / CONFIG.MAX_DISTANCE_MM * r;
        ctx.beginPath();
        ctx.arc(cx, cy, rr, 0, 2 * Math.PI);
        ctx.strokeStyle = d <= CONFIG.DANGER_DISTANCE_MM ? CONFIG.C.dangerLine
            : d <= CONFIG.SAFE_DISTANCE_MM ? CONFIG.C.safeLine : CONFIG.C.grid;
        ctx.lineWidth = (d === CONFIG.DANGER_DISTANCE_MM || d === CONFIG.SAFE_DISTANCE_MM) ? 1 : 0.5;
        if (ctx.lineWidth === 1) ctx.setLineDash([5, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillText((d / 1000) + 'm', cx + rr + 3, cy + 3);
    });

    ctx.strokeStyle = CONFIG.C.grid;
    ctx.lineWidth = 0.3;
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let a = 0; a < 360; a += 30) {
        const rad = a * D2R;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + r * Math.sin(rad), cy - r * Math.cos(rad));
        ctx.stroke();
        ctx.fillText(a + '°', cx + (r + 16) * Math.sin(rad), cy - (r + 16) * Math.cos(rad));
    }
}

function drawDirection() {
    if (!cartCmd) return;
    let dx = cartCmd.dx || 0, dy = cartCmd.dy || 0;
    const total = Math.hypot(dx, dy);
    if (total < 1) return;
    const scale = Math.min(total, 6000) / 6000 * r;
    const nx = dy / total * scale;
    const ny = -dx / total * scale;
    const ex = cx + nx, ey = cy + ny;
    const g = ctx.createLinearGradient(cx, cy, ex, ey);
    g.addColorStop(0, 'transparent');
    g.addColorStop(0.7, 'rgba(0,200,255,0.06)');
    g.addColorStop(1, 'rgba(0,200,255,0.18)');
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = g;
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawPoints() {
    const points = radarData.map(p => ({
        pt: toCanvas(p.a, p.d),
        color: getColor(p.d)
    }));
    points.forEach(({pt, color}) => {
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(pt.x, pt.y);
        ctx.strokeStyle = hexToRgba(color, 0.10);
        ctx.lineWidth = 0.5;
        ctx.stroke();
    });
    points.forEach(({pt, color}) => {
        ctx.save();
        ctx.shadowBlur = 2;
        ctx.shadowColor = hexToRgba(color, 0.5);
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 2, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.restore();
    });
}

function loop() {
    try {
        ctx.clearRect(0, 0, w, h);
        drawZoneFill(CONFIG.DANGER_DISTANCE_MM, CONFIG.C.dangerFill);
        drawZoneRing(CONFIG.DANGER_DISTANCE_MM, CONFIG.SAFE_DISTANCE_MM, CONFIG.C.safeFill);
        drawGrid();
        drawPoints();
        drawDirection();
    } catch (e) {
        console.error('Render error:', e);
    }
    requestAnimationFrame(loop);
}

// --- Connection health ---

const els = {
    status: document.querySelector('.status'),
    statusText: document.getElementById('statusText'),
    fpsText: document.getElementById('fpsText'),
    temp: document.getElementById('tempDisplay'),
    statDanger: document.getElementById('statDanger'),
    statSafe: document.getElementById('statSafe'),
    statTotal: document.getElementById('statTotal')
};

let reconnectDelay = 1000;
let reconnectTimer = null;
let lastDataTime = 0;
let staleNotified = false;
let fpsTimestamps = [];

function updateFps() {
    const now = Date.now();
    lastDataTime = now;
    if (staleNotified) {
        staleNotified = false;
        els.fpsText.style.color = '';
    }
    fpsTimestamps.push(now);
    while (fpsTimestamps.length > 0 && now - fpsTimestamps[0] > 1000)
        fpsTimestamps.shift();
    els.fpsText.textContent = fpsTimestamps.length + ' frame/s';
}

function checkStale() {
    const now = Date.now();
    while (fpsTimestamps.length > 0 && now - fpsTimestamps[0] > 1000)
        fpsTimestamps.shift();
    if (!isConnected || lastDataTime === 0) return;
    if (fpsTimestamps.length > 0) return;
    const elapsed = now - lastDataTime;
    if (elapsed > 600) {
        if (!staleNotified) {
            staleNotified = true;
            els.fpsText.textContent = 'refreshing';
            els.fpsText.style.color = '#ffaa00';
        }
        if (ws && ws.readyState === WebSocket.OPEN) ws.close();
    } else if (elapsed > 300 && !staleNotified) {
        staleNotified = true;
        els.fpsText.textContent = 'refreshing';
        els.fpsText.style.color = '#ffaa00';
    }
}

function connect() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    setState('connecting');
    try {
        ws = new WebSocket(`ws://${location.hostname}:80/ws`);
        ws.onopen = () => {
            isConnected = true;
            reconnectDelay = 1000;
            setState('connected');
        };
        ws.onmessage = e => { try { handle(JSON.parse(e.data)); } catch (err) { console.error('WS handle error:', err, e.data); } };
        ws.onerror = () => setState('error');
        ws.onclose = () => {
            isConnected = false;
            ws = null;
            lastDataTime = 0;
            fpsTimestamps.length = 0;
            setState('connecting');
            reconnectTimer = setTimeout(() => {
                reconnectDelay = Math.min(reconnectDelay * 2, 5000);
                connect();
            }, reconnectDelay);
        };
    } catch (_) { lastDataTime = 0; setState('connecting'); }
}

function setState(s) {
    els.status.className = 'status ' + (s === 'error' ? 'error' : s === 'connected' ? 'connected' : 'connecting');
    els.statusText.textContent = s === 'connected' ? '已连接'
        : s === 'error' ? '连接错误' : '连接中';
}

function handle(data) {
    if (data.temp != null && !isNaN(data.temp))
        els.temp.textContent = data.temp.toFixed(1) + ' °C';
    if (data.cart)
        cartCmd = data.cart;
    const raw = data.vectors || [];
    const filtered = [];
    let danger = 0, safe = 0;
    for (let i = 0; i < raw.length; i++) {
        const d = raw[i].d;
        if (d <= 0 || isNaN(d) || d > CONFIG.MAX_DISTANCE_MM) continue;
        filtered.push(raw[i]);
        if (d <= CONFIG.DANGER_DISTANCE_MM) danger++;
        else if (d <= CONFIG.SAFE_DISTANCE_MM) safe++;
    }
    radarData = filtered;
    els.statDanger.textContent = danger;
    els.statSafe.textContent = safe;
    els.statTotal.textContent = filtered.length;
    updateFps();
}

window.onload = function() {
    init();
    connect();
    setInterval(checkStale, 200);
};
