/**

  Moveo

  - Copyright (C) 2024 - PRESENT  rookidroid.com
  - E-mail: info@rookidroid.com
  - Website: https://rookidroid.com/

                        **
                       ****
                        **
                        **
                        **
                        **

        **********************************
      **************************************
     ****************************************
     ********      ************      ********
     *******        **********        *******
     *******        **********        *******
     ********      ************      ********
     ****************************************
     ****************************************
     ****************************************
     ****************************************


            **************************

                ******************

*/

/** Stepper Motor */
#include "FastAccelStepper.h"

/** Servo */
#include <ESP32Servo.h>

/** WiFi */
#include <AsyncUDP.h>
#include <WiFi.h>
#include <esp_wifi.h>  // for esp_wifi_set_ps() / WIFI_PS_NONE

/** Async Web Server
 *  Install via Arduino Library Manager:
 *    - "ESPAsyncWebServer" by lacamera (or me-no-dev)
 *    - "AsyncTCP" by dvarrel (or me-no-dev)  ← required dependency
 */
#include <ESPAsyncWebServer.h>

/** FreeRTOS queue (thread-safe bridge between Core 0 web callbacks and Core 1 loop) */
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/** OTA */
#include <ArduinoOTA.h>

/* ─────────────────────────────────────────────
   Joint Stepper Pin Configuration
   ───────────────────────────────────────────── */
#define J1_DIR  13
#define J1_STEP 12

#define J2_DIR  14
#define J2_STEP 27

#define J3_DIR  26
#define J3_STEP 25

#define J4_DIR  33
#define J4_STEP 32

#define J5_DIR  15
#define J5_STEP 2

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper *j1_stepper = NULL;
FastAccelStepper *j2_stepper = NULL;
FastAccelStepper *j3_stepper = NULL;
FastAccelStepper *j4_stepper = NULL;
FastAccelStepper *j5_stepper = NULL;

/* ─────────────────────────────────────────────
   Hand Servo Configuration
   ───────────────────────────────────────────── */
#define servoPin 4
const int SERVO_MID = 1500;
const int SERVO_MIN = 700;
const int SERVO_MAX = 2300;

Servo hand_servo;
volatile int servo_us = SERVO_MID;

/* ─────────────────────────────────────────────
   WiFi Configuration  (Access Point)
   ───────────────────────────────────────────── */
#ifndef APSSID
  #define APSSID "moveo"
  #define APPSK  "moveo_1234"
#endif

const char *ssid     = APSSID;
const char *password = APPSK;

AsyncUDP udp_socket;

/* ─────────────────────────────────────────────
   FreeRTOS Command Queue
   Web handlers (Core 0) post commands here;
   loop() (Core 1) drains and executes them.
   ───────────────────────────────────────────── */
enum CmdType : uint8_t {
  CMD_MOVE,    // joint + steps (relative)
  CMD_MOVETO,  // joint + pos   (absolute)
  CMD_CONFIG,  // joint + speed + accel
  CMD_STOP,    // stop all
  CMD_HOME,    // home all
  CMD_SERVO    // servo us
};

struct Command {
  CmdType type;
  int     joint;  // 1-5 for steppers
  int32_t val1;   // steps / pos / speed / us
  int32_t val2;   // accel (CMD_CONFIG only)
};

static QueueHandle_t cmdQueue;

/* ─────────────────────────────────────────────
   Async Web Server  (non-blocking, Core 0)
   ───────────────────────────────────────────── */
AsyncWebServer server(80);

/* ── Embedded control page (PROGMEM) ── */
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Moveo Control</title>
<style>
  :root{
    --bg:#0d1117;--surface:#161b22;--surface2:#21262d;
    --accent:#58a6ff;--accent2:#3fb950;--danger:#f85149;
    --warn:#e3b341;--text:#e6edf3;--muted:#8b949e;
    --radius:12px;--gap:16px;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;padding:20px;}
  h1{text-align:center;font-size:1.8rem;font-weight:700;letter-spacing:.05em;
     background:linear-gradient(90deg,var(--accent),var(--accent2));
     -webkit-background-clip:text;-webkit-text-fill-color:transparent;margin-bottom:6px;}
  .subtitle{text-align:center;color:var(--muted);font-size:.85rem;margin-bottom:24px;}
  .top-bar{display:flex;gap:12px;justify-content:center;margin-bottom:28px;flex-wrap:wrap;}
  button{cursor:pointer;border:none;border-radius:8px;font-size:.85rem;font-weight:600;
         padding:9px 18px;transition:transform .1s,opacity .15s;}
  button:active{transform:scale(.95);}
  button:disabled{opacity:.4;cursor:not-allowed;}
  .btn-danger{background:var(--danger);color:#fff;}
  .btn-home{background:var(--surface2);color:var(--text);border:1px solid var(--muted);}
  .btn-primary{background:var(--accent);color:#0d1117;}
  .btn-secondary{background:var(--surface2);color:var(--text);border:1px solid var(--muted);}
  .btn-apply{background:var(--warn);color:#0d1117;font-size:.78rem;padding:7px 14px;}
  .btn-home:hover,.btn-secondary:hover{background:var(--surface);}
  .btn-primary:hover{opacity:.85;}
  .btn-danger:hover{opacity:.85;}

  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:var(--gap);}
  .card{background:var(--surface);border:1px solid var(--surface2);border-radius:var(--radius);padding:18px;
        display:flex;flex-direction:column;gap:12px;}
  .card-title{font-size:1rem;font-weight:700;display:flex;align-items:center;gap:8px;}
  .badge{font-size:.7rem;padding:3px 8px;border-radius:20px;background:var(--surface2);color:var(--muted);}
  .badge.stepper{border:1px solid var(--accent);color:var(--accent);}
  .badge.servo{border:1px solid var(--accent2);color:var(--accent2);}

  .pos-display{background:var(--bg);border-radius:8px;padding:10px 14px;font-size:1.4rem;
               font-weight:700;letter-spacing:.04em;text-align:center;color:var(--accent);
               border:1px solid var(--surface2);}

  label{font-size:.75rem;color:var(--muted);display:block;margin-bottom:4px;}
  input[type=number],input[type=range]{
    width:100%;background:var(--bg);border:1px solid var(--surface2);border-radius:6px;
    color:var(--text);padding:7px 10px;font-size:.85rem;outline:none;
    transition:border-color .2s;}
  input[type=number]:focus{border-color:var(--accent);}
  input[type=range]{padding:4px 0;accent-color:var(--accent2);}

  .row{display:flex;gap:8px;align-items:flex-end;}
  .row > div{flex:1;}
  .row > button{flex-shrink:0;}

  .config-row{display:grid;grid-template-columns:1fr 1fr auto;gap:8px;align-items:flex-end;}

  .divider{border:none;border-top:1px solid var(--surface2);}
  .servo-val{text-align:center;font-size:.9rem;color:var(--accent2);font-weight:600;margin-top:2px;}

  .conn-dot{display:inline-block;width:8px;height:8px;border-radius:50%;
            background:var(--muted);margin-right:6px;transition:background .4s;}
  .conn-dot.ok{background:var(--accent2);}
  .conn-dot.err{background:var(--danger);}

  .toast{position:fixed;bottom:24px;right:24px;background:var(--surface2);border:1px solid var(--surface);
         border-radius:8px;padding:12px 18px;font-size:.85rem;opacity:0;transform:translateY(10px);
         transition:opacity .3s,transform .3s;pointer-events:none;z-index:999;max-width:280px;}
  .toast.show{opacity:1;transform:translateY(0);}
  .toast.ok{border-left:3px solid var(--accent2);}
  .toast.err{border-left:3px solid var(--danger);}

  @media(max-width:480px){.config-row{grid-template-columns:1fr 1fr;} .config-row button{grid-column:1/-1;}}
</style>
</head>
<body>

<h1>&#129470; Moveo Control</h1>
<p class="subtitle">
  <span class="conn-dot" id="dot"></span>
  <span id="conn-status">Connecting…</span> &bull; 192.168.4.1
</p>

<div class="top-bar">
  <button class="btn-danger" id="btnStop" onclick="stopAll()">&#9632; EMERGENCY STOP</button>
  <button class="btn-home"   id="btnHome" onclick="homeAll()">&#8962; Home All</button>
</div>

<div class="grid" id="grid"></div>
<div class="toast" id="toast"></div>

<script>
const JOINTS = [
  {id:'j1', label:'Joint 1', type:'stepper'},
  {id:'j2', label:'Joint 2', type:'stepper'},
  {id:'j3', label:'Joint 3', type:'stepper'},
  {id:'j4', label:'Joint 4', type:'stepper'},
  {id:'j5', label:'Joint 5', type:'stepper'},
  {id:'j6', label:'Hand Servo (J6)', type:'servo'},
];

// ── Build cards ───────────────────────────────────────────────────────────────
const grid = document.getElementById('grid');
JOINTS.forEach(j => {
  if(j.type === 'stepper'){
    grid.innerHTML += `
    <div class="card" id="card-${j.id}">
      <div class="card-title">${j.label} <span class="badge stepper">Stepper</span></div>
      <div class="pos-display" id="pos-${j.id}">— steps</div>
      <hr class="divider"/>
      <div>
        <label>Move by steps</label>
        <div class="row">
          <div><input type="number" id="steps-${j.id}" value="100" min="1"/></div>
          <button class="btn-primary"   onclick="move('${j.id}', 1)">+ Move</button>
          <button class="btn-secondary" onclick="move('${j.id}',-1)">&minus; Move</button>
        </div>
      </div>
      <div>
        <label>Move to absolute position</label>
        <div class="row">
          <div><input type="number" id="abs-${j.id}" value="0"/></div>
          <button class="btn-primary" onclick="moveTo('${j.id}')">&#8594; Go</button>
        </div>
      </div>
      <hr class="divider"/>
      <div>
        <label>Configuration</label>
        <div class="config-row">
          <div>
            <label>Speed (Hz)</label>
            <input type="number" id="speed-${j.id}" value="3000" min="1"/>
          </div>
          <div>
            <label>Accel (steps/s&sup2;)</label>
            <input type="number" id="accel-${j.id}" value="800" min="1"/>
          </div>
          <button class="btn-apply" onclick="applyConfig('${j.id}')">&#10003; Apply</button>
        </div>
      </div>
    </div>`;
  } else {
    grid.innerHTML += `
    <div class="card" id="card-${j.id}">
      <div class="card-title">${j.label} <span class="badge servo">Servo</span></div>
      <div class="pos-display" id="pos-${j.id}">— &micro;s</div>
      <hr class="divider"/>
      <div>
        <label>Pulse Width (700 &ndash; 2300 &micro;s)</label>
        <input type="range" id="slider-j6" min="700" max="2300" value="1500"
               oninput="updateServoLabel(this.value)" onchange="sendServo(this.value)"/>
        <div class="servo-val" id="servo-label">1500 &micro;s</div>
      </div>
    </div>`;
  }
});

// ── Connection state ──────────────────────────────────────────────────────────
const dot  = document.getElementById('dot');
const csts = document.getElementById('conn-status');
let connected = false;
function setConnected(ok){
  connected = ok;
  dot.className  = 'conn-dot ' + (ok ? 'ok' : 'err');
  csts.textContent = ok ? 'Connected' : 'Disconnected';
}

// ── API helpers ───────────────────────────────────────────────────────────────
const TIMEOUT_MS = 8000;

async function api(path, body){
  const ctrl = new AbortController();
  const tid  = setTimeout(() => ctrl.abort(), TIMEOUT_MS);
  try {
    const opts = body !== undefined
      ? {method:'POST', headers:{'Content-Type':'application/json'},
         body:JSON.stringify(body), signal:ctrl.signal}
      : {method:'GET', signal:ctrl.signal};
    const r = await fetch(path, opts);
    clearTimeout(tid);
    if(!r.ok) throw new Error('HTTP ' + r.status);
    return await r.json();
  } catch(e){
    clearTimeout(tid);
    const msg = e.name === 'AbortError' ? 'Request timed out' : e.message;
    showToast(msg, 'err');
    setConnected(false);
    return null;
  }
}

function showToast(msg, type='ok'){
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'toast show ' + type;
  clearTimeout(t._timer);
  t._timer = setTimeout(() => t.className='toast', 2800);
}

// ── Actions ───────────────────────────────────────────────────────────────────
const jIndex = id => parseInt(id.replace('j',''));

async function move(id, dir){
  const steps = parseInt(document.getElementById('steps-'+id).value) * dir;
  const r = await api('/move', {joint: jIndex(id), steps});
  if(r){ setConnected(true); showToast(`${id.toUpperCase()} moved ${steps>0?'+':''}${steps} steps`); }
}

async function moveTo(id){
  const pos = parseInt(document.getElementById('abs-'+id).value);
  const r = await api('/moveto', {joint: jIndex(id), pos});
  if(r){ setConnected(true); showToast(`${id.toUpperCase()} \u2192 ${pos}`); }
}

async function applyConfig(id){
  const speed = parseInt(document.getElementById('speed-'+id).value);
  const accel = parseInt(document.getElementById('accel-'+id).value);
  const r = await api('/config', {joint: jIndex(id), speed, accel});
  if(r){ setConnected(true); showToast(`${id.toUpperCase()} config applied`); }
}

async function stopAll(){
  const r = await api('/stop', {});
  if(r){ setConnected(true); showToast('All motors stopped', 'err'); }
}

async function homeAll(){
  const r = await api('/home', {});
  if(r){ setConnected(true); showToast('Homing all joints\u2026'); }
}

function updateServoLabel(v){
  document.getElementById('servo-label').textContent = v + ' \u00b5s';
}

async function sendServo(v){
  const r = await api('/servo', {us: parseInt(v)});
  if(r){ setConnected(true); showToast('Servo \u2192 ' + v + ' \u00b5s'); }
}

// ── Sequential status polling (no pile-up) ────────────────────────────────────
let polling = false;
async function pollStatus(){
  if(polling) return;
  polling = true;
  const ctrl = new AbortController();
  const tid  = setTimeout(() => ctrl.abort(), TIMEOUT_MS);
  try {
    const r = await fetch('/status', {signal: ctrl.signal});
    clearTimeout(tid);
    if(!r.ok) throw new Error();
    const d = await r.json();
    setConnected(true);
    for(let i=1;i<=5;i++){
      const el = document.getElementById('pos-j'+i);
      if(el) el.textContent = (d['j'+i] ?? '?') + ' steps';
    }
    const sv = document.getElementById('pos-j6');
    if(sv) sv.textContent = (d.servo ?? '?') + ' \u00b5s';
  } catch(_){
    clearTimeout(tid);
    setConnected(false);
  } finally {
    polling = false;
  }
}

// Start polling with a fixed interval; sequential guard prevents pile-up
setInterval(pollStatus, 2000);
pollStatus();
</script>
</body>
</html>
)rawhtml";

/* ─────────────────────────────────────────────
   Helper: get FastAccelStepper* by joint index
   ───────────────────────────────────────────── */
FastAccelStepper* stepperByIndex(int idx) {
  switch (idx) {
    case 1: return j1_stepper;
    case 2: return j2_stepper;
    case 3: return j3_stepper;
    case 4: return j4_stepper;
    case 5: return j5_stepper;
    default: return nullptr;
  }
}

/* ─────────────────────────────────────────────
   Minimal JSON integer extractor
   e.g. jsonInt("{\"joint\":2,\"steps\":100}", "steps") → 100
   ───────────────────────────────────────────── */
int jsonInt(const String& body, const char* key) {
  String pattern = String("\"") + key + "\"";
  int idx = body.indexOf(pattern);
  if (idx < 0) return 0;
  idx += pattern.length();
  while (idx < (int)body.length() && (body[idx] == ':' || body[idx] == ' ')) idx++;
  int end = idx;
  if (end < (int)body.length() && body[end] == '-') end++;
  while (end < (int)body.length() && isDigit(body[end])) end++;
  return body.substring(idx, end).toInt();
}

/* ─────────────────────────────────────────────
   Helper: add CORS headers to every response
   ───────────────────────────────────────────── */
void addCors(AsyncWebServerResponse* resp) {
  resp->addHeader("Access-Control-Allow-Origin",  "*");
  resp->addHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  resp->addHeader("Access-Control-Allow-Headers", "Content-Type");
}

/* ─────────────────────────────────────────────
   Async REST Route Handlers
   All callbacks run in the lwIP/WiFi task (Core 0)
   — never block here, never call delay().
   ───────────────────────────────────────────── */

// GET /  →  serve the control page
void handleRoot(AsyncWebServerRequest* request) {
  AsyncWebServerResponse* resp = request->beginResponse_P(200, "text/html", INDEX_HTML);
  resp->addHeader("Cache-Control", "no-cache");
  request->send(resp);
}

// GET /status  →  JSON with current positions
void handleStatus(AsyncWebServerRequest* request) {
  String json = "{";
  for (int i = 1; i <= 5; i++) {
    FastAccelStepper* s = stepperByIndex(i);
    long pos = s ? s->getCurrentPosition() : 0;
    json += "\"j" + String(i) + "\":" + String(pos);
    if (i < 5) json += ",";
  }
  json += ",\"servo\":" + String(servo_us) + "}";
  AsyncWebServerResponse* resp = request->beginResponse(200, "application/json", json);
  addCors(resp);
  request->send(resp);
}

// POST /move  →  body {joint, steps}
// Enqueues CMD_MOVE; motor call executes in loop() on Core 1.
void handleMove(AsyncWebServerRequest* request,
                uint8_t* data, size_t len, size_t /*index*/, size_t /*total*/) {
  String body  = String((char*)data, len);
  int jIdx     = jsonInt(body, "joint");
  int steps    = jsonInt(body, "steps");
  if (jIdx < 1 || jIdx > 5) { request->send(404, "application/json", "{\"error\":\"joint not found\"}"); return; }
  Command cmd  = {CMD_MOVE, jIdx, (int32_t)steps, 0};
  xQueueSend(cmdQueue, &cmd, 0);
  request->send(200, "application/json", "{\"ok\":true}");
}

// POST /moveto  →  body {joint, pos}
void handleMoveTo(AsyncWebServerRequest* request,
                  uint8_t* data, size_t len, size_t /*index*/, size_t /*total*/) {
  String body  = String((char*)data, len);
  int jIdx     = jsonInt(body, "joint");
  int pos      = jsonInt(body, "pos");
  if (jIdx < 1 || jIdx > 5) { request->send(404, "application/json", "{\"error\":\"joint not found\"}"); return; }
  Command cmd  = {CMD_MOVETO, jIdx, (int32_t)pos, 0};
  xQueueSend(cmdQueue, &cmd, 0);
  request->send(200, "application/json", "{\"ok\":true}");
}

// POST /config  →  body {joint, speed, accel}
void handleConfig(AsyncWebServerRequest* request,
                  uint8_t* data, size_t len, size_t /*index*/, size_t /*total*/) {
  String body  = String((char*)data, len);
  int jIdx     = jsonInt(body, "joint");
  int speed    = jsonInt(body, "speed");
  int accel    = jsonInt(body, "accel");
  if (jIdx < 1 || jIdx > 5) { request->send(404, "application/json", "{\"error\":\"joint not found\"}"); return; }
  Command cmd  = {CMD_CONFIG, jIdx, (int32_t)speed, (int32_t)accel};
  xQueueSend(cmdQueue, &cmd, 0);
  request->send(200, "application/json", "{\"ok\":true}");
}

// POST /stop  →  stop all motors immediately
void handleStop(AsyncWebServerRequest* request) {
  Command cmd = {CMD_STOP, 0, 0, 0};
  xQueueSend(cmdQueue, &cmd, 0);
  request->send(200, "application/json", "{\"ok\":true}");
}

// POST /home  →  move all steppers to position 0
void handleHome(AsyncWebServerRequest* request) {
  Command cmd = {CMD_HOME, 0, 0, 0};
  xQueueSend(cmdQueue, &cmd, 0);
  request->send(200, "application/json", "{\"ok\":true}");
}

// POST /servo  →  body {us}
void handleServo(AsyncWebServerRequest* request,
                 uint8_t* data, size_t len, size_t /*index*/, size_t /*total*/) {
  String body  = String((char*)data, len);
  int us       = constrain(jsonInt(body, "us"), SERVO_MIN, SERVO_MAX);
  servo_us     = us;  // update the readable state immediately
  Command cmd  = {CMD_SERVO, 0, (int32_t)us, 0};
  xQueueSend(cmdQueue, &cmd, 0);
  request->send(200, "application/json", "{\"ok\":true}");
}

/* ─────────────────────────────────────────────
   setup()
   ───────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);

  // ── WiFi AP ──────────────────────────────────
  WiFi.persistent(false);          // avoid unnecessary flash writes
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  // Disable ALL power-saving modes — the #1 cause of ESP32 AP dropouts.
  // Modem sleep lets the radio go quiet between beacons; under motor ISR
  // load the radio sometimes misses its wake window and the AP vanishes.
  WiFi.setSleep(false);                    // Arduino-level modem sleep off
  esp_wifi_set_ps(WIFI_PS_NONE);           // IDF-level power saving off
  WiFi.setTxPower(WIFI_POWER_19_5dBm);    // maximum TX power

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // ── OTA ──────────────────────────────────────
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
      // using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  // ── GPIO ─────────────────────────────────────
  pinMode(J1_DIR,  OUTPUT);
  pinMode(J1_STEP, OUTPUT);
  pinMode(J2_DIR,  OUTPUT);
  pinMode(J2_STEP, OUTPUT);
  pinMode(J3_DIR,  OUTPUT);
  pinMode(J3_STEP, OUTPUT);
  pinMode(J4_DIR,  OUTPUT);
  pinMode(J4_STEP, OUTPUT);
  pinMode(J5_DIR,  OUTPUT);
  pinMode(J5_STEP, OUTPUT);

  // ── Servo ─────────────────────────────────────
  hand_servo.attach(servoPin, SERVO_MIN, SERVO_MAX);
  hand_servo.writeMicroseconds(SERVO_MID);

  // ── Steppers ──────────────────────────────────
  engine.init();

  j1_stepper = engine.stepperConnectToPin(J1_STEP);
  if (j1_stepper) {
    j1_stepper->setDirectionPin(J1_DIR);
    j1_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j1_stepper->setAcceleration(800);  // 800 steps/s²
  }

  j2_stepper = engine.stepperConnectToPin(J2_STEP);
  if (j2_stepper) {
    j2_stepper->setDirectionPin(J2_DIR);
    j2_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j2_stepper->setAcceleration(800);  // 800 steps/s²
  }

  j3_stepper = engine.stepperConnectToPin(J3_STEP);
  if (j3_stepper) {
    j3_stepper->setDirectionPin(J3_DIR);
    j3_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j3_stepper->setAcceleration(800);  // 800 steps/s²
  }

  j4_stepper = engine.stepperConnectToPin(J4_STEP);
  if (j4_stepper) {
    j4_stepper->setDirectionPin(J4_DIR);
    j4_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j4_stepper->setAcceleration(800);  // 800 steps/s²
  }

  j5_stepper = engine.stepperConnectToPin(J5_STEP);
  if (j5_stepper) {
    j5_stepper->setDirectionPin(J5_DIR);
    j5_stepper->setSpeedInHz(3000);    // 3000 steps/s
    j5_stepper->setAcceleration(800);  // 800 steps/s²
  }

  // ── FreeRTOS command queue ─────────────────────
  // Depth 20 — more than enough for burst button presses
  cmdQueue = xQueueCreate(20, sizeof(Command));

  // ── Async Web Server routes ───────────────────
  // GET endpoints
  server.on("/",       HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);

  // POST endpoints — body handled via the onBody lambda (3rd arg)
  // Signature: handler(request), upload(request,…), body(request, data, len, index, total)
  server.on("/move", HTTP_POST,
    [](AsyncWebServerRequest* r){ /* ACK sent inside body handler */ },
    NULL,
    handleMove);

  server.on("/moveto", HTTP_POST,
    [](AsyncWebServerRequest* r){ },
    NULL,
    handleMoveTo);

  server.on("/config", HTTP_POST,
    [](AsyncWebServerRequest* r){ },
    NULL,
    handleConfig);

  server.on("/stop", HTTP_POST,
    handleStop);

  server.on("/home", HTTP_POST,
    handleHome);

  server.on("/servo", HTTP_POST,
    [](AsyncWebServerRequest* r){ },
    NULL,
    handleServo);

  // CORS pre-flight (OPTIONS) — reply 204 for all paths
  server.onNotFound([](AsyncWebServerRequest* request) {
    if (request->method() == HTTP_OPTIONS) {
      AsyncWebServerResponse* resp = request->beginResponse(204);
      addCors(resp);
      request->send(resp);
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });

  server.begin();
  Serial.println("Async web server started on http://192.168.4.1");
}

/* ─────────────────────────────────────────────
   loop()  — runs on Core 1
   Drains the command queue so all motor / servo
   API calls happen here, away from Core 0 lwIP.
   ───────────────────────────────────────────── */
void loop() {
  ArduinoOTA.handle();

  // ── AP watchdog ───────────────────────────────
  // Restart the softAP if its IP disappears (rare but possible under heavy
  // motor ISR load). Checked every 5 s to avoid any overhead.
  static uint32_t lastApCheck = 0;
  uint32_t now = millis();
  if (now - lastApCheck >= 5000) {
    lastApCheck = now;
    if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
      Serial.println("[watchdog] AP down — restarting softAP");
      WiFi.softAP(ssid, password);
      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);
    }
  }

  // ── Drain command queue ───────────────────────
  Command cmd;
  while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {

      case CMD_MOVE: {
        FastAccelStepper* s = stepperByIndex(cmd.joint);
        if (s) s->move(cmd.val1);
        break;
      }

      case CMD_MOVETO: {
        FastAccelStepper* s = stepperByIndex(cmd.joint);
        if (s) s->moveTo(cmd.val1);
        break;
      }

      case CMD_CONFIG: {
        FastAccelStepper* s = stepperByIndex(cmd.joint);
        if (s) {
          if (cmd.val1 > 0) s->setSpeedInHz(cmd.val1);
          if (cmd.val2 > 0) s->setAcceleration(cmd.val2);
        }
        break;
      }

      case CMD_STOP:
        for (int i = 1; i <= 5; i++) {
          FastAccelStepper* s = stepperByIndex(i);
          if (s) s->stopMove();
        }
        break;

      case CMD_HOME:
        for (int i = 1; i <= 5; i++) {
          FastAccelStepper* s = stepperByIndex(i);
          if (s) s->moveTo(0);
        }
        break;

      case CMD_SERVO:
        hand_servo.writeMicroseconds((int)cmd.val1);
        break;
    }
  }

  // Yield to FreeRTOS scheduler so the WiFi task (Core 0) and other
  // system tasks get CPU time every loop iteration.
  vTaskDelay(1);
}
