/**
 * ESP32 Ridar Viewer Logic
 */

const CONFIG = {
    MAX_DISTANCE_MM: 6900,
    DANGER_DISTANCE_MM: 1000,
    SAFE_DISTANCE_MM: 6000
};

let chart = null;
let ws = null;

// --- 初始化图表 ---
function initChart() {
    const dom = document.getElementById('chart-container');
    chart = echarts.init(dom, null, { renderer: 'canvas' });

    // 辅助函数：生成同心圆网格线数据
    const generateCircle = (r) => Array.from({ length: 72 }, (_, i) => ({ value: [i * 5, r] }));

    const seriesList = [
        // 1. 网格线 (同心圆)
        {
            type: 'line',
            coordinateSystem: 'polar',
            data: generateCircle(CONFIG.DANGER_DISTANCE_MM),
            symbol: 'none',
            lineStyle: { color: '#ff4444', type: 'dashed', width: 1, opacity: 0.35 }
        },
        {
            type: 'line',
            coordinateSystem: 'polar',
            data: generateCircle(CONFIG.SAFE_DISTANCE_MM),
            symbol: 'none',
            lineStyle: { color: '#44ff44', type: 'dashed', width: 1, opacity: 0.35 }
        },
        // 2. 危险区域填充 (0-1m)
        {
            type: 'custom',
            coordinateSystem: 'polar',
            z: 1,
            silent: true,
            data: [[0, 0, CONFIG.DANGER_DISTANCE_MM]],
            renderItem: (params, api) => renderZone(api, 0, CONFIG.DANGER_DISTANCE_MM, 'rgba(255,68,68,0.08)')
        },
        // 3. 安全区域填充 (1-6m)
        {
            type: 'custom',
            coordinateSystem: 'polar',
            z: 1,
            silent: true,
            data: [[0, CONFIG.DANGER_DISTANCE_MM, CONFIG.SAFE_DISTANCE_MM]],
            renderItem: (params, api) => renderZone(api, CONFIG.DANGER_DISTANCE_MM, CONFIG.SAFE_DISTANCE_MM, 'rgba(68,255,68,0.04)')
        },
        // 4. 当前雷达点数据 (核心修改在此处)
        {
            id: 'radar-points',
            type: 'scatter',
            coordinateSystem: 'polar',
            symbolSize: 10,
            // 【新增】显式编码维度，防止 ECharts 搞混角度和半径
            encode: {
                angle: 0, // 强制指定数据数组的第一位是角度
                radius: 1 // 强制指定数据数组的第二位是半径
            },
            itemStyle: {
                shadowBlur: 10,
                shadowColor: 'rgba(0, 212, 255, 0.5)'
            },
            animationDurationUpdate: 100
        }
    ];

    const option = {
        backgroundColor: 'transparent',
        animation: false,
        polar: {
            center: ['50%', '50%'],
            radius: '80%'
        },
        angleAxis: {
            type: 'value',
            startAngle: 90,          // 0° 指向正北
            clockwise: true,         // 顺时针增加
            min: 0,
            max: 360,
            axisLine: { show: false },
            axisLabel: {
                color: '#aaa',
                fontSize: 10,
                formatter: '{value}°'
            },
            splitLine: { lineStyle: { color: '#222', width: 0.5, opacity: 0.3 } }
        },
        radiusAxis: {
            type: 'value',
            min: 0,
            max: CONFIG.MAX_DISTANCE_MM,
            axisLine: { show: false },
            axisLabel: {
                color: '#aaa',
                fontSize: 10,
                formatter: v => (v / 1000).toFixed(0) + 'm'
            },
            splitLine: { lineStyle: { color: '#222', width: 0.5, opacity: 0.3 } }
        },
        gridIndex: 0,
        series: seriesList
    };

    chart.setOption(option);
}

// --- 绘制扇形区域 (与 Python fill_between 效果一致) ---
function renderZone(api, innerR, outerR, color) {
    const steps = 72;
    const points = [];
    for (let i = 0; i <= steps; i++) {
        const a = i * 360 / steps;
        points.push(api.coord([a, outerR])); // API会自动处理 [angle, radius] -> canvas x,y
    }
    for (let i = steps; i >= 0; i--) {
        const a = i * 360 / steps;
        points.push(api.coord([a, innerR]));
    }
    return {
        type: 'polygon',
        shape: { points },
        style: { fill: color },
        silent: true
    };
}

// --- WebSocket 连接管理 ---
const els = {
    ip: document.getElementById('ipInput'),
    port: document.getElementById('portInput'),
    btn: document.getElementById('connectBtn'),
    statusDot: document.querySelector('#statusArea .dot'),
    statusText: document.getElementById('statusText')
};

function toggleConnection() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        disconnect();
    } else {
        connect();
    }
}

function connect() {
    const ip = els.ip.value.trim();
    const port = els.port.value.trim();
    const url = `ws://${ip}:${port}/`;

    setStatus('connecting');

    try {
        ws = new WebSocket(url);

        ws.onopen = () => {
            setStatus('connected');
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                updateChartData(data);
            } catch (e) {
                console.error("Parse Error", e);
            }
        };

        ws.onerror = (err) => {
            console.error(err);
            setStatus('error');
        };

        ws.onclose = () => {
            setStatus('disconnected');
        };

    } catch (e) {
        alert("Invalid URL or IP.");
    }
}

function disconnect() {
    if (ws) {
        ws.close();
        ws = null;
    }
}

function setStatus(state) {
    switch (state) {
        case 'connected':
            els.btn.textContent = '断开';
            els.btn.className = 'btn-disconnect';
            els.statusDot.parentElement.className = 'status connected';
            els.statusText.textContent = '已连接';
            break;
        case 'connecting':
            els.btn.textContent = '...';
            els.btn.disabled = true;
            els.statusDot.parentElement.className = 'status connecting';
            els.statusText.textContent = '连接中...';
            break;
        default:
            els.btn.textContent = '连接';
            els.btn.className = 'btn-connect';
            els.btn.disabled = false;
            els.statusDot.parentElement.className = 'status disconnected';
            els.statusText.textContent = state === 'error' ? '连接错误' : '未连接';
            break;
    }
}

// --- 数据处理与展示 ---
function updateChartData(jsonData) {
    const vectors = jsonData.vectors || [];

    // 1. 数据清洗：保留有效距离 (排除 0, NaN, 以及传感器故障值 65535)
    const validVectors = vectors.filter(v =>
        v.d > 0 &&
        !isNaN(v.d) &&
        v.d !== 65535 &&
        v.d <= CONFIG.MAX_DISTANCE_MM
    );

    // 2. 格式化数据
    // 此时结构变为: { value: [角度, 距离], itemStyle: {...} }
    const radarPoints = validVectors.map(v => ({
        value: [v.a, v.d],
        itemStyle: { color: getPointColor(v.d) }
    }));

    // 3. 更新图表
    if (chart) {
        chart.setOption({
            series: [{ id: 'radar-points', data: radarPoints }]
        });
    }

    // 4. 更新底部原始数据显示
    const viewer = document.getElementById('json-viewer');
    viewer.textContent = `[${new Date().toLocaleTimeString()}]\n` + JSON.stringify(jsonData, null, 2);
    viewer.scrollTop = viewer.scrollHeight;
}

// --- 颜色判断 (近红远绿) ---
function getPointColor(dist) {
    if (dist <= CONFIG.DANGER_DISTANCE_MM) return '#ff4444';
    if (dist <= CONFIG.SAFE_DISTANCE_MM) return '#44ff44';
    return '#66d9ef';
}

// --- 启动 ---
window.onload = () => {
    initChart();
    window.addEventListener('resize', () => chart.resize());
    els.btn.addEventListener('click', toggleConnection);

    [els.ip, els.port].forEach(el => el.addEventListener('keypress', e => {
        if (e.key === 'Enter') toggleConnection();
    }));
};
