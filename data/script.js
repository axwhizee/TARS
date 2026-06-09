const D2R = Math.PI / 180;
const MAX = 5000, SAFE = 4000, DANGER = 800, PMIN = 100;
const DEAD_RATIO = 0.05;
const JS_SEND_MS = 125;

const CLR = {
    red:    '#ff4444',
    green:  '#44ff44',
    yellow: '#ffaa00',
    blue:   '#55bbff',
    text:   '#c0c0d0',
    grid:   'rgba(200,200,220,0.15)',
};

let radarData = [], cartCmd = null;
let ws = null, isConnected = false;
let canvas, ctx, w, h, cx, cy, r;

let isManualMode = true, jsActive = false;
let joystickCmd = { dx: 0, dy: 0 }, lastSendMs = 0;

function hexToRgba(hex, a) {
    const i = parseInt(hex.slice(1), 16);
    return `rgba(${(i >> 16) & 0xff},${(i >> 8) & 0xff},${i & 0xff},${a})`;
}

function getColor(d) {
    if (d < PMIN)   return CLR.yellow;
    if (d <= DANGER) return CLR.red;
    if (d <= SAFE)   return CLR.green;
    return CLR.yellow;
}

function toCanvasRad(deg, dist) {
    const rad = deg * D2R;
    return { x: cx + dist / MAX * r * Math.sin(rad), y: cy - dist / MAX * r * Math.cos(rad) };
}

function sendWs(data) {
    if (ws && ws.readyState === WebSocket.OPEN)
        try { ws.send(JSON.stringify(data)); } catch (_) {}
}

function sendCmdNow() {
    const c = { dx: joystickCmd.dx, dy: joystickCmd.dy };
    if (c.dx < 0) c.dy = -c.dy;
    sendWs({ cmd: c });
}

function sendResetJoystick() {
    jsActive = false;
    joystickCmd = { dx: 0, dy: 0 };
    sendCmdNow();
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
    const B = canvas.parentElement.getBoundingClientRect();
    w = B.width; h = B.height;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    canvas.style.width = w + 'px';
    canvas.style.height = h + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    cx = w / 2; cy = h / 2;
    r = Math.min(cx, cy) * 0.82;
}

function toR(mm) { return mm / MAX * r; }

function fillRing(inner, outer, style) {
    ctx.beginPath();
    ctx.arc(cx, cy, toR(outer), 0, 2 * Math.PI);
    if (inner) ctx.arc(cx, cy, toR(inner), 0, 2 * Math.PI, true);
    ctx.fillStyle = style;
    ctx.fill();
}

function strokeRing(mm, style, lw, dash) {
    ctx.beginPath();
    ctx.arc(cx, cy, toR(mm), 0, 2 * Math.PI);
    ctx.strokeStyle = style;
    ctx.lineWidth = lw;
    if (dash) ctx.setLineDash(dash);
    ctx.stroke();
    ctx.setLineDash([]);
}

function drawBackdrop() {
    fillRing(0, DANGER, hexToRgba(CLR.red, 0.06));
    fillRing(DANGER, SAFE, hexToRgba(CLR.green, 0.03));
    strokeRing(DANGER, hexToRgba(CLR.red, 0.30), 1, [4, 4]);
    strokeRing(SAFE,   hexToRgba(CLR.green, 0.30), 1, [4, 4]);
    strokeRing(MAX,    CLR.grid, 1, null);

    ctx.fillStyle = CLR.text;
    ctx.font = '9px monospace';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    for (const d of [1000, 2000, 3000, 4000]) {
        strokeRing(d, CLR.grid, 0.7);
        ctx.fillText((d / 1000) + 'm', cx + toR(d) + 3, cy + 3);
    }

    ctx.strokeStyle = CLR.grid;
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
    const dx = isManualMode ? joystickCmd.dx : (cartCmd ? cartCmd.dx : 0);
    const dy = isManualMode ? joystickCmd.dy : (cartCmd ? cartCmd.dy : 0);
    const mag = Math.hypot(dx, dy);
    if (mag < 1) return;
    const scale = Math.min(mag, MAX) / MAX * r;
    const ex = cx + dy / mag * scale;
    const ey = cy - dx / mag * scale;
    const base = isManualMode ? CLR.red : CLR.blue;
    const g = ctx.createLinearGradient(cx, cy, ex, ey);
    g.addColorStop(0, 'transparent');
    g.addColorStop(0.7, hexToRgba(base, 0.25));
    g.addColorStop(1, hexToRgba(base, 0.9));
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = g;
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawPoints() {
    const len = radarData.length;
    for (let i = 0; i < len; i++) {
        const p = radarData[i];
        const pt = toCanvasRad(p.a, p.d);
        const color = getColor(p.d);
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(pt.x, pt.y);
        ctx.strokeStyle = hexToRgba(color, 0.10);
        ctx.lineWidth = 0.5;
        ctx.stroke();
        const rgba = hexToRgba(color, 0.5);
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 3, 0, 2 * Math.PI);
        ctx.fillStyle = rgba;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 2, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
    }
}

function loop() {
    ctx.clearRect(0, 0, w, h);
    drawBackdrop();
    drawPoints();
    drawDirection();
    requestAnimationFrame(loop);
}

const els = {
    status:     document.querySelector('.status'),
    statusText: document.getElementById('statusText'),
    fpsText:    document.getElementById('fpsText'),
    temp:       document.getElementById('tempDisplay'),
    statNoise:  document.getElementById('statNoise'),
    statDanger: document.getElementById('statDanger'),
    statSafe:   document.getElementById('statSafe'),
    statTotal:  document.getElementById('statTotal')
};

let reconnectDelay = 1000, reconnectTimer = null;
let lastDataTime = 0, staleNotified = false, fpsTimestamps = [];

function updateFps() {
    const now = Date.now();
    lastDataTime = now;
    if (staleNotified) { staleNotified = false; els.fpsText.style.color = ''; }
    fpsTimestamps.push(now);
    while (fpsTimestamps.length > 0 && now - fpsTimestamps[0] > 1000) fpsTimestamps.shift();
    els.fpsText.textContent = fpsTimestamps.length + ' frame/s';
}

function checkStale() {
    if (!isConnected || lastDataTime === 0) return;
    const now = Date.now();
    while (fpsTimestamps.length > 0 && now - fpsTimestamps[0] > 1000) fpsTimestamps.shift();
    if (fpsTimestamps.length > 0) return;
    if (!staleNotified) { staleNotified = true; els.fpsText.textContent = 'refreshing'; els.fpsText.style.color = CLR.yellow; }
    if (now - lastDataTime > 600 && ws && ws.readyState === WebSocket.OPEN) ws.close();
}

function setState(s) {
    els.status.className = 'status ' + (s === 'error' ? 'error' : s === 'connected' ? 'connected' : 'connecting');
    els.statusText.textContent = s === 'connected' ? '已连接' : s === 'error' ? '连接错误' : '连接中';
}

function connect() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    setState('connecting');
    try {
        ws = new WebSocket(`ws://${location.hostname}:80/ws`);
        ws.onopen = () => { isConnected = true; reconnectDelay = 1000; setState('connected'); sendWs({ mode: 'manual' }); };
        ws.onmessage = e => { try { handle(JSON.parse(e.data)); } catch (_) {} };
        ws.onerror = () => setState('error');
        ws.onclose = () => {
            isConnected = false; ws = null; lastDataTime = 0; fpsTimestamps.length = 0;
            setState('connecting');
            reconnectTimer = setTimeout(() => { reconnectDelay = Math.min(reconnectDelay * 2, 5000); connect(); }, reconnectDelay);
        };
    } catch (_) { setState('connecting'); }
}

function handle(data) {
    if (data.temp != null && !isNaN(data.temp)) {
        const t = data.temp;
        els.temp.textContent = t.toFixed(1) + ' °C';
        els.temp.style.color = t < 30 ? CLR.green : t < 50 ? CLR.yellow : CLR.red;
    }
    if (data.cart) cartCmd = data.cart;
    const raw = data.vectors || [], filtered = [];
    let noise = 0, danger = 0, safe = 0;
    for (let i = 0; i < raw.length; i++) {
        const d = raw[i].d;
        if (d <= 0 || isNaN(d) || MAX < d) { noise++; continue; }
        if (d < PMIN || SAFE < d) noise++;
        else if (d <= DANGER) danger++;
        else safe++;
        filtered.push(raw[i]);
    }
    radarData = filtered;
    els.statNoise.textContent = noise;
    els.statDanger.textContent = danger;
    els.statSafe.textContent = safe;
    els.statTotal.textContent = raw.length;
    updateFps();
}

function updateJoystick(e) {
    const B = canvas.getBoundingClientRect();
    const rawDx = -(e.clientY - B.top - cy) / r * MAX;
    const rawDy =  (e.clientX - B.left - cx) / r * MAX;
    const dist = Math.hypot(rawDx, rawDy);
    if (dist < MAX * DEAD_RATIO) { joystickCmd = { dx: 0, dy: 0 }; }
    else { const s = dist > MAX ? MAX / dist : 1; joystickCmd = { dx: rawDx * s, dy: rawDy * s }; }
    const now = Date.now();
    if (now - lastSendMs < JS_SEND_MS) return;
    lastSendMs = now;
    sendCmdNow();
}

window.onload = function() {
    init();
    connect();
    setInterval(checkStale, 200);

    document.getElementById('modeToggle').addEventListener('click', () => {
        isManualMode = !isManualMode;
        const btn = document.getElementById('modeToggle');
        btn.textContent = isManualMode ? '手动' : '自动';
        btn.classList.toggle('manual', isManualMode);
        sendWs({ mode: isManualMode ? 'manual' : 'auto' });
        if (!isManualMode) sendResetJoystick();
    });
    document.getElementById('modeToggle').classList.add('manual');

    function onPointerDown(e) {
        if (!isManualMode) return;
        jsActive = true;
        canvas.setPointerCapture(e.pointerId);
        updateJoystick(e);
    }
    function onPointerMove(e) { if (isManualMode && jsActive) updateJoystick(e); }
    function onPointerUp()    { if (isManualMode) sendResetJoystick(); }

    canvas.addEventListener('pointerdown', onPointerDown);
    canvas.addEventListener('pointermove', onPointerMove);
    canvas.addEventListener('pointerup',   onPointerUp);
    canvas.addEventListener('pointercancel', onPointerUp);

    canvas.addEventListener('touchstart', e => e.preventDefault(), { passive: false });
    canvas.addEventListener('touchmove',  e => e.preventDefault(), { passive: false });

    function onPageHide() { if (isManualMode) sendResetJoystick(); }
    document.addEventListener('visibilitychange', onPageHide);
    window.addEventListener('blur', onPageHide);
};
