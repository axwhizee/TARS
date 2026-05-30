const CONFIG = {
    MAX_DISTANCE_MM: 5000,
    SAFE_DISTANCE_MM: 4000,
    DANGER_DISTANCE_MM: 800,
    PERCEPTION_MIN: 100,
    GRID_STEPS: [1000, 2000, 3000, 4000],
    C: {
        grid: '#3c3c5c58',
        text: '#888',
        dangerFill: 'rgba(255,68,68,0.06)',
        safeFill: 'rgba(68,255,68,0.03)',
        dangerLine: 'rgba(255,68,68,0.30)',
        safeLine: 'rgba(68,255,68,0.30)',
        pointDanger: '#ff4444',
        pointSafe: '#44ff44',
        pointNoise: '#888888'
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
    if (d < CONFIG.PERCEPTION_MIN)        return CONFIG.C.pointNoise;
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

function toR(mm) { return mm / CONFIG.MAX_DISTANCE_MM * r; }

function fillRing(innerMM, outerMM, style) {
    ctx.beginPath();
    ctx.arc(cx, cy, toR(outerMM), 0, 2 * Math.PI);
    if (innerMM > 0) ctx.arc(cx, cy, toR(innerMM), 0, 2 * Math.PI, true);
    ctx.fillStyle = style;
    ctx.fill();
}

function strokeRing(mm, style, w, dash) {
    ctx.beginPath();
    ctx.arc(cx, cy, toR(mm), 0, 2 * Math.PI);
    ctx.strokeStyle = style;
    ctx.lineWidth = w;
    if (dash) ctx.setLineDash(dash);
    ctx.stroke();
    ctx.setLineDash([]);
}

function drawBackdrop() {
    fillRing(0, CONFIG.DANGER_DISTANCE_MM, CONFIG.C.dangerFill);
    fillRing(CONFIG.DANGER_DISTANCE_MM, CONFIG.SAFE_DISTANCE_MM, CONFIG.C.safeFill);

    strokeRing(CONFIG.DANGER_DISTANCE_MM, CONFIG.C.dangerLine, 1, [4, 4]);
    strokeRing(CONFIG.SAFE_DISTANCE_MM,   CONFIG.C.safeLine,   1, [4, 4]);
    strokeRing(CONFIG.MAX_DISTANCE_MM,    CONFIG.C.grid,       1, null);

    ctx.fillStyle = CONFIG.C.text;
    ctx.font = '9px monospace';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    CONFIG.GRID_STEPS.forEach(d => {
        strokeRing(d, CONFIG.C.grid, 0.7);
        ctx.fillText((d / 1000) + 'm', cx + toR(d) + 3, cy + 3);
    });

    ctx.strokeStyle = CONFIG.C.grid;
    ctx.lineWidth = 0.5;
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
    const scale = Math.min(total, CONFIG.MAX_DISTANCE_MM) / CONFIG.MAX_DISTANCE_MM * r;
    const nx = dy / total * scale;
    const ny = -dx / total * scale;
    const ex = cx + nx, ey = cy + ny;
    const g = ctx.createLinearGradient(cx, cy, ex, ey);
    g.addColorStop(0, 'transparent');
    g.addColorStop(0.7, 'rgba(0, 60, 80, 0.25)');
    g.addColorStop(1, 'rgba(255, 70, 0, 0.8)');
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = g;
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawPoints() {
    radarData.forEach(p => {
        const pt = toCanvas(p.a, p.d);
        const color = getColor(p.d);
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(pt.x, pt.y);
        ctx.strokeStyle = hexToRgba(color, 0.10);
        ctx.lineWidth = 0.5;
        ctx.stroke();
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
        drawBackdrop();
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
    statNoise: document.getElementById('statNoise'),
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
    let noise = 0, danger = 0, safe = 0;
    for (let i = 0; i < raw.length; i++) {
        const d = raw[i].d;
        if (d <= 0 || isNaN(d) || d > CONFIG.MAX_DISTANCE_MM) { noise++; continue; }
        if (d < CONFIG.PERCEPTION_MIN || d > CONFIG.SAFE_DISTANCE_MM) {
            noise++;
        } else if (d <= CONFIG.DANGER_DISTANCE_MM) {
            danger++;
        } else {
            safe++;
        }
        filtered.push(raw[i]);
    }
    radarData = filtered;
    els.statNoise.textContent = noise;
    els.statDanger.textContent = danger;
    els.statSafe.textContent = safe;
    els.statTotal.textContent = raw.length;
    updateFps();
}

window.onload = function() {
    init();
    connect();
    setInterval(checkStale, 200);
};
