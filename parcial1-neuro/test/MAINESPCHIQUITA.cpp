#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Adafruit_NeoPixel.h>

// ========== CONFIGURACIÓN WiFi ==========
const char* ssid     = "Robotsito";
const char* password = "12345678";

// ========== PINES ==========
#define PIN_RGB  48
#define RX_PIN    1   // GPIO1 → G22 (TX) ESP grande
#define TX_PIN    2   // GPIO2 → G36 (RX) ESP grande

// ========== HARDWARE ==========
Adafruit_NeoPixel pixel(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
WebSocketsServer webServer(81);
WiFiServer server(80);

// ========== HTML DASHBOARD ==========
const char* html_page = R"=====(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Neurorobot — Telemetría</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@300;400;600&family=IBM+Plex+Sans:wght@300;500&display=swap');

  :root {
    --bg:       #080c10;
    --bg2:      #0d1218;
    --bg3:      #111820;
    --border:   #1e2a38;
    --accent:   #00d4ff;
    --green:    #00ff88;
    --orange:   #ff8c00;
    --red:      #ff3344;
    --dim:      #3a4a5a;
    --text:     #c8d8e8;
    --text2:    #5a7080;
    --HIST: 200;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'IBM Plex Mono', monospace;
    font-size: 12px;
    min-height: 100vh;
    overflow-x: hidden;
  }

  header {
    display: flex; align-items: center; gap: 16px;
    padding: 10px 18px;
    background: var(--bg2);
    border-bottom: 1px solid var(--border);
    position: sticky; top: 0; z-index: 100;
  }
  header .logo { font-size: 11px; letter-spacing: 3px; color: var(--accent); text-transform: uppercase; }
  header .logo span { color: var(--dim); }

  #conn-badge {
    padding: 3px 10px; border: 1px solid var(--dim);
    font-size: 10px; letter-spacing: 1px; color: var(--dim); transition: all .3s;
  }
  #conn-badge.connected { border-color: var(--green); color: var(--green); }
  #conn-badge.error     { border-color: var(--red);   color: var(--red);   }

  #state-badge {
    margin-left: auto; padding: 4px 16px;
    font-size: 13px; letter-spacing: 2px; font-weight: 600;
    background: transparent; border: 1px solid var(--dim); color: var(--dim); transition: all .3s;
  }
  #state-badge.CRUCERO      { border-color: var(--green);  color: var(--green);  }
  #state-badge.GIRANDO      { border-color: var(--orange); color: var(--orange); background: #1a0d00; }
  #state-badge.STOP         { border-color: var(--red);    color: var(--red);    background: #1a0008; }
  #state-badge.RUGOSO-RECTO { border-color: var(--orange); color: var(--orange); }
  #state-badge.IMU-DETECT   { border-color: #cc88ff;       color: #cc88ff; }

  #time-label { font-size: 10px; color: var(--dim); white-space: nowrap; }

  .main-grid {
    display: grid;
    grid-template-columns: 280px 1fr;
    grid-template-rows: auto 1fr;
    gap: 0;
    min-height: calc(100vh - 45px);
  }

  .sidebar {
    background: var(--bg2); border-right: 1px solid var(--border);
    padding: 14px; display: flex; flex-direction: column; gap: 14px; grid-row: 1 / 3;
  }

  .panel-title {
    font-size: 9px; letter-spacing: 2px; color: var(--dim);
    text-transform: uppercase; margin-bottom: 8px;
    padding-bottom: 4px; border-bottom: 1px solid var(--border);
  }

  .sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
  .scard { background: var(--bg3); border: 1px solid var(--border); padding: 8px 10px; }
  .scard .slabel { font-size: 9px; color: var(--dim); letter-spacing: 1px; margin-bottom: 2px; }
  .scard .sval   { font-size: 20px; font-weight: 600; color: var(--accent); line-height: 1; }
  .scard .sunit  { font-size: 9px; color: var(--dim); margin-left: 2px; }
  .scard.warn   .sval { color: var(--orange); }
  .scard.danger .sval { color: var(--red); }

  .imu-row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 5px; }
  .icard { background: var(--bg3); border: 1px solid var(--border); padding: 7px 8px; }
  .icard .ilabel { font-size: 9px; color: var(--dim); }
  .icard .ival   { font-size: 16px; font-weight: 600; color: var(--text); }

  .log-box {
    flex: 1; background: #050810; border: 1px solid var(--border);
    overflow-y: auto; padding: 6px 8px; font-size: 10px; line-height: 1.6;
  }
  .log-line .lts  { color: #1a2a22; margin-right: 6px; }
  .log-line .ltag { color: #1a6a3a; margin-right: 4px; }
  .log-line .ldat { color: #2a6a4a; }
  .log-line:last-child .ldat { color: #00cc66; }

  .right-panel { display: flex; flex-direction: column; background: var(--bg); }

  .neuro-section { padding: 14px 16px 10px; border-bottom: 1px solid var(--border); }
  .neuro-grid    { display: grid; grid-template-columns: repeat(5, 1fr); gap: 8px; }

  .ncell {
    background: var(--bg2); border: 1px solid var(--border);
    padding: 8px 10px; position: relative; overflow: hidden;
  }
  .ncell .ntag  { font-size: 9px; letter-spacing: 1px; color: var(--dim); }
  .ncell .ndesc { font-size: 8px; color: var(--dim); opacity: .6; margin-bottom: 3px; }
  .ncell .nval  {
    font-size: 24px; font-weight: 600;
    color: var(--cell-color, var(--dim)); line-height: 1; transition: color .1s;
  }
  .ncell .nbar-bg   { height: 3px; background: var(--border); margin-top: 4px; }
  .ncell .nbar-fill {
    height: 100%; background: var(--cell-color, var(--dim));
    width: 0%; transition: width .15s ease;
  }

  .chart-section {
    flex: 1; padding: 10px 16px 14px;
    display: flex; flex-direction: column; gap: 10px;
  }
  .chart-wrap { background: var(--bg2); border: 1px solid var(--border); padding: 8px 10px 6px; flex: 1; }
  .chart-header { display: flex; align-items: center; gap: 14px; margin-bottom: 6px; }
  .chart-title  { font-size: 9px; letter-spacing: 2px; color: var(--dim); text-transform: uppercase; }
  .legend       { display: flex; gap: 10px; flex-wrap: wrap; }
  .legend-item  { display: flex; align-items: center; gap: 4px; font-size: 9px; color: var(--dim); }
  .legend-dot   { width: 8px; height: 2px; display: inline-block; }

  canvas { display: block; width: 100%; image-rendering: pixelated; }
</style>
</head>
<body>

<header>
  <div class="logo">⬡ <span>NEURO</span>ROBOT <span>//</span> TELEMETRÍA</div>
  <div id="conn-badge">● DESCONECTADO</div>
  <div id="state-badge">—</div>
  <div id="time-label">t = 0 ms</div>
</header>

<div class="main-grid">

  <aside class="sidebar">
    <div>
      <div class="panel-title">Ultrasonidos</div>
      <div class="sensor-grid">
        <div class="scard" id="card-us1">
          <div class="slabel">FRONTAL</div>
          <div class="sval" id="v-us1">—</div><span class="sunit">cm</span>
        </div>
        <div class="scard" id="card-us2">
          <div class="slabel">OBJETIVO</div>
          <div class="sval" id="v-us2">—</div><span class="sunit">cm</span>
        </div>
      </div>
    </div>

    <div>
      <div class="panel-title">IMU</div>
      <div class="imu-row">
        <div class="icard"><div class="ilabel">ROLL</div><div class="ival" id="v-roll">—</div></div>
        <div class="icard"><div class="ilabel">PITCH</div><div class="ival" id="v-pitch">—</div></div>
        <div class="icard"><div class="ilabel">YAW</div><div class="ival" id="v-yaw">—</div></div>
      </div>
    </div>

    <div>
      <div class="panel-title">Terminal [MINI-S3]</div>
      <div class="log-box" id="log-box"></div>
    </div>
  </aside>

  <div class="right-panel">
    <div class="neuro-section">
      <div class="panel-title" style="margin-bottom:10px">Activación neuronal</div>
      <div class="neuro-grid" id="neuro-grid"></div>
    </div>

    <div class="chart-section">
      <div class="chart-wrap" style="flex:2">
        <div class="chart-header">
          <div class="chart-title">Historial N0–N6 + Mem</div>
          <div class="legend" id="legend-n"></div>
        </div>
        <canvas id="chart-n" height="140"></canvas>
      </div>

      <div class="chart-wrap" style="flex:1">
        <div class="chart-header">
          <div class="chart-title">N3 Parada · Atr · Arb</div>
          <div class="legend" id="legend-k"></div>
        </div>
        <canvas id="chart-k" height="90"></canvas>
      </div>
    </div>
  </div>
</div>

<script>
const HIST = 200;

// ── CAMBIO 1: claves actualizadas al formato del nuevo main.cpp ──
// CSV: ST:...,DF:...,OBJ:...,R:...,P:...,Y:...,
//      N0:...,N1:...,N2:...,N3:...,N4:...,N5:...,N6:...,
//      MEM:...,ATR:...,ARB:...,GIRO:...
const NEURO_DEFS = [
  { key:'N0',  label:'N0',  desc:'Terreno',    color:'#4477dd' },
  { key:'N1',  label:'N1',  desc:'Prox. izq',  color:'#1D9E75' },
  { key:'N2',  label:'N2',  desc:'Prox. der',  color:'#0F6E56' },
  { key:'N3',  label:'N3',  desc:'Parada',     color:'#e03030' },
  { key:'N4',  label:'N4',  desc:'Integrador', color:'#c47a00' },
  { key:'N5',  label:'N5',  desc:'Osc A',      color:'#8877ee' },
  { key:'N6',  label:'N6',  desc:'Osc B',      color:'#dd5588' },
  { key:'MEM', label:'Mem', desc:'Memoria',    color:'#4a9e20' },
  { key:'ATR', label:'Atr', desc:'Atracción',  color:'#00d4ff' },
  { key:'ARB', label:'Arb', desc:'Árbitro',    color:'#ff8c00' },
];

const hist = {};
NEURO_DEFS.forEach(d => hist[d.key] = new Float32Array(HIST));
let histPtr = 0;
let tickMs  = 0;
let lastState = '';

const grid   = document.getElementById('neuro-grid');
const nCells = {};
NEURO_DEFS.forEach(d => {
  const cell = document.createElement('div');
  cell.className = 'ncell';
  cell.style.setProperty('--cell-color', d.color);
  cell.innerHTML = `
    <div class="ntag">${d.label}</div>
    <div class="ndesc">${d.desc}</div>
    <div class="nval" id="nv-${d.key}">0.0</div>
    <div class="nbar-bg"><div class="nbar-fill" id="nb-${d.key}"></div></div>
  `;
  grid.appendChild(cell);
  nCells[d.key] = {
    val: document.getElementById(`nv-${d.key}`),
    bar: document.getElementById(`nb-${d.key}`),
  };
});

function buildLegend(id, defs) {
  const el = document.getElementById(id);
  defs.forEach(d => {
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.innerHTML = `<span class="legend-dot" style="background:${d.color}"></span>${d.label}`;
    el.appendChild(item);
  });
}
buildLegend('legend-n', NEURO_DEFS.slice(0, 8));
buildLegend('legend-k', [
  { label:'N3', color:'#e03030' },
  { label:'Atr',color:'#00d4ff' },
  { label:'Arb',color:'#ff8c00' },
]);

function drawChart(canvasId, keys, colors, yMax) {
  const canvas = document.getElementById(canvasId);
  const W = canvas.parentElement.clientWidth - 20;
  const H = canvas.height;
  canvas.width = W;
  const ctx = canvas.getContext('2d');

  ctx.fillStyle = '#0d1218';
  ctx.fillRect(0, 0, W, H);

  ctx.strokeStyle = '#1e2a38';
  ctx.lineWidth   = 1;
  [0.25, 0.5, 0.75, 1.0].forEach(f => {
    const y = Math.round(H * (1 - f));
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  });

  const PAD = 2, pw = W - PAD * 2, ph = H - PAD * 2;

  keys.forEach((key, i) => {
    ctx.beginPath();
    ctx.strokeStyle = colors[i];
    ctx.lineWidth   = 1.5;
    ctx.shadowColor = colors[i];
    ctx.shadowBlur  = 3;
    for (let j = 0; j < HIST; j++) {
      const idx = (histPtr + j) % HIST;
      const v   = hist[key][idx];
      const x   = PAD + (j / (HIST - 1)) * pw;
      const y   = PAD + ph * (1 - Math.min(v, yMax) / yMax);
      j === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
  });
}

function redrawAll() {
  drawChart('chart-n',
    ['N0','N1','N2','N3','N4','N5','N6','MEM'],
    ['#4477dd','#1D9E75','#0F6E56','#e03030','#c47a00','#8877ee','#dd5588','#4a9e20'],
    100
  );
  drawChart('chart-k',
    ['N3','ATR','ARB'],
    ['#e03030','#00d4ff','#ff8c00'],
    100
  );
}

function parseCSV(raw) {
  const out = {};
  raw.split(',').forEach(tok => {
    const p = tok.indexOf(':');
    if (p > -1) out[tok.substring(0, p).trim()] = tok.substring(p + 1).trim();
  });
  return out;
}

function applyData(d) {
  const st = (d['ST'] || '—').trim();
  if (st !== lastState) {
    const badge = document.getElementById('state-badge');
    badge.textContent = st;
    badge.className   = '';
    badge.classList.add(st.split(' ')[0]);
    lastState = st;
  }

  // ── CAMBIO 2: DF y OBJ en lugar de US1 y US2 ──
  function setUS(cardId, valId, key) {
    const v    = parseFloat(d[key]);
    const el   = document.getElementById(valId);
    const card = document.getElementById(cardId);
    if (!isNaN(v)) {
      el.textContent = v >= 999 ? '---' : v.toFixed(1);
      card.className = 'scard' + (v <= 5 ? ' danger' : v <= 20 ? ' warn' : '');
    }
  }
  setUS('card-us1', 'v-us1', 'DF');
  setUS('card-us2', 'v-us2', 'OBJ');

  ['roll','pitch','yaw'].forEach(k => {
    const map = { roll:'R', pitch:'P', yaw:'Y' };
    const v   = parseFloat(d[map[k]]);
    const el  = document.getElementById('v-' + k);
    if (!isNaN(v)) el.textContent = v.toFixed(1) + '°';
  });

  // ── CAMBIO 3: itera sobre NEURO_DEFS en lugar del zMap de 3 claves ──
  NEURO_DEFS.forEach(def => {
    const v = parseFloat(d[def.key]);
    if (!isNaN(v)) {
      hist[def.key][histPtr] = v;
      const c = nCells[def.key];
      c.val.textContent = v.toFixed(1);
      c.bar.style.width = Math.min(v, 100) + '%';
    }
  });

  // ── CAMBIO 4: eliminado el bloque que copiaba Z1,Z2,Z4,Z5,Z6 ──
  // Ahora todas las neuronas llegan en el CSV, no hay que rellenar
  histPtr = (histPtr + 1) % HIST;
  tickMs += 150;
  document.getElementById('time-label').textContent = `t = ${tickMs} ms`;

  redrawAll();
}

function addLog(raw) {
  const box  = document.getElementById('log-box');
  const line = document.createElement('div');
  line.className = 'log-line';
  const ts = new Date().toLocaleTimeString('es-CO');
  line.innerHTML = `<span class="lts">${ts}</span><span class="ltag">[S3]</span><span class="ldat">${raw}</span>`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
  if (box.children.length > 200) box.removeChild(box.firstChild);
}

const badge = document.getElementById('conn-badge');
let ws, wsRetry = 0;

function connect() {
  ws = new WebSocket('ws://' + window.location.hostname + ':81/');

  ws.onopen = () => {
    badge.textContent = '● CONECTADO';
    badge.className   = 'connected';
    wsRetry = 0;
  };

  ws.onclose = () => {
    badge.textContent = '● DESCONECTADO';
    badge.className   = '';
    wsRetry++;
    setTimeout(connect, Math.min(wsRetry * 1000, 5000));
  };

  ws.onerror = () => {
    badge.textContent = '● ERROR WS';
    badge.className   = 'error';
  };

  ws.onmessage = (ev) => {
    const raw = ev.data;
    addLog(raw);
    if (raw.startsWith('ST:')) {
      applyData(parseCSV(raw));
    }
  };
}

connect();
window.addEventListener('resize', redrawAll);
redrawAll();
</script>
</body>
</html>
)=====";

// ========== VARIABLES ==========
unsigned long lastPixelUpdate = 0;

void setPixel(uint8_t r, uint8_t g, uint8_t b) {
    if (millis() - lastPixelUpdate < 300) return;
    lastPixelUpdate = millis();
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("\n=== MINI ESP32-S3 NEUROBOT TELEMETRIA ===");

    pixel.begin();
    pixel.setBrightness(10);
    pixel.setPixelColor(0, pixel.Color(80, 40, 0));
    pixel.show();

    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial1.setTimeout(50);
    Serial.printf("Serial1: RX=GPIO%d TX=GPIO%d\n", RX_PIN, TX_PIN);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.printf("AP: %s  IP: %s\n", ssid, WiFi.softAPIP().toString().c_str());

    server.begin();
    webServer.begin();

    webServer.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        if (type == WStype_CONNECTED) {
            Serial.printf("WS cliente #%d conectado\n", num);
            pixel.setPixelColor(0, pixel.Color(0, 0, 30));
            pixel.show();
        }
        if (type == WStype_DISCONNECTED) {
            pixel.setPixelColor(0, pixel.Color(0, 20, 0));
            pixel.show();
        }
    });

    pixel.setPixelColor(0, pixel.Color(0, 20, 0));
    pixel.show();
    Serial.println("Listo — abre http://192.168.4.1 en el navegador\n");
}

void loop() {
    webServer.loop();

    WiFiClient client = server.available();
    if (client && client.connected()) {
        unsigned long t0 = millis();
        while (!client.available() && millis() - t0 < 50) {
            webServer.loop();
            delay(1);
        }
        if (client.available()) {
            client.readStringUntil('\r');
            client.flush();
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html; charset=utf-8");
            client.println("Connection: close");
            client.println();
            client.print(html_page);
            client.stop();
            Serial.println("Página servida");
        }
    }

    if (Serial1.available()) {
        String data = Serial1.readStringUntil('\n');
        data.trim();
        if (data.length() > 0) {
            Serial.print("[RX] ");
            Serial.println(data);
            webServer.broadcastTXT(data);
            setPixel(0, 30, 0);
        }
    }

    delay(5);
}