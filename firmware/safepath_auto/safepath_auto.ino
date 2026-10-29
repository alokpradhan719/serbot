/*
  =====================================================================
   SafePath Robot - AUTO firmware (ESP32)
  =====================================================================
   The ESP32 is only the "muscles + reflexes".  The laptop dashboard is
   the "brain": it builds the map, plans paths and decides every move.

   What the robot does by itself:
     - creates Wi-Fi "SafePath_Robot" (password SafePath123)
     - executes simple timed commands from the laptop (drive / turn / scan)
     - EMERGENCY BRAKE: stops on its own if an obstacle gets too close
       while driving (no waiting for Wi-Fi - this is the reflex)
     - stops on its own if the laptop stops talking to it

   The old manual phone dashboard still works at http://192.168.4.1

   Laptop API (all GET, JSON replies, CORS enabled):
     /api/ping
     /api/status
     /api/drive?dir=F|B&ms=800&spd=180&safe=18&id=N
     /api/turn?dir=L|R&ms=500&spd=200&id=N
     /api/scan?a=15,50,90,130,165&id=N
     /api/stop

   Library needed: "ESP32Servo" by Kevin Harrington


   WIRING (same as before - nothing changes)
     L298N: IN1=26 IN2=27 IN3=14 IN4=25 ENA=33 ENB=32 (ENA/ENB jumpers removed)
     Servo: signal=23     Front SR04 (on servo): TRIG=4  ECHO=18 (divider)
                          Back SR04 (fixed):     TRIG=19 ECHO=21 (divider)
  =====================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char* ssid = "SafePath_Robot";
const char* password = "SafePath123";
WebServer server(80);

// ---------- Pins ----------
const int IN1 = 26, IN2 = 27, IN3 = 14, IN4 = 25;
const int ENA = 33, ENB = 32;
const int SERVO_PIN = 23;
const int TRIG_F = 4,  ECHO_F = 18;   // front sensor (on servo)
const int TRIG_B = 19, ECHO_B = 21;   // back sensor (fixed)

// ---------- Settings ----------
const int STOP_DISTANCE = 20;            // cm (manual mode)
const int SERVO_CENTER = 90;             // adjust if sensor isn't straight at 90
const int SERVO_LEFT   = 160;            // swap with SERVO_RIGHT if mirrored
const int SERVO_RIGHT  = 20;
const unsigned long COMMAND_TIMEOUT = 700;
const unsigned long LAPTOP_TIMEOUT = 3000;   // stop if laptop silent this long mid-move

// ---------- State ----------
Servo myServo;
int servoAngle = SERVO_CENTER;
float distFront = -1, distBack = -1;
float scanLeft = -1, scanCenter = -1, scanRight = -1;
String scanResult = "Not scanned yet";

unsigned long lastSensorRead = 0;
bool readFrontNext = true;
unsigned long lastCommandTime = 0;
char currentDir = 'S';

int scanStep = 0;                 // manual scan state machine (0 = idle)
unsigned long scanStepTime = 0;

// ---------- Motor speed (ENA / ENB PWM) ----------
ESP32PWM pwmA, pwmB;
int motorSpeed = 150;
const int MIN_SPEED = 110;
const int TURN_PERCENT = 75;
const int LEFT_TRIM  = 100;       // lower the faster motor if robot drifts
const int RIGHT_TRIM = 100;

void applySpeed(int spd) {
  pwmA.write(constrain(spd * LEFT_TRIM / 100, 0, 255));
  pwmB.write(constrain(spd * RIGHT_TRIM / 100, 0, 255));
}

// ---------- Motors ----------
void setMotors(int a, int b, int c, int d) {
  digitalWrite(IN1, a); digitalWrite(IN2, b);
  digitalWrite(IN3, c); digitalWrite(IN4, d);
}
void stopMotors() { applySpeed(0); setMotors(LOW, LOW, LOW, LOW); currentDir = 'S'; }
void forward()    { setMotors(HIGH, LOW, HIGH, LOW); applySpeed(motorSpeed); currentDir = 'F'; }
void reverse()    { setMotors(LOW, HIGH, LOW, HIGH); applySpeed(motorSpeed); currentDir = 'B'; }
void turnLeft()   { setMotors(LOW, HIGH, HIGH, LOW); applySpeed(max(MIN_SPEED, motorSpeed * TURN_PERCENT / 100)); currentDir = 'L'; }
void turnRight()  { setMotors(HIGH, LOW, LOW, HIGH); applySpeed(max(MIN_SPEED, motorSpeed * TURN_PERCENT / 100)); currentDir = 'R'; }

bool frontBlocked() {
  return abs(servoAngle - SERVO_CENTER) <= 15 && distFront > 0 && distFront < STOP_DISTANCE;
}
bool backBlocked() { return distBack > 0 && distBack < STOP_DISTANCE; }

// ---------- Sensors ----------
float readDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

// Median of 5 readings - removes random ultrasonic spikes
float medianRead(int trig, int echo) {
  float v[5];
  for (int i = 0; i < 5; i++) {
    v[i] = readDistance(trig, echo);
    if (v[i] < 0) v[i] = 9999;
    delay(10);
  }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4 - i; j++)
      if (v[j] > v[j + 1]) { float t = v[j]; v[j] = v[j + 1]; v[j + 1] = t; }
  return (v[2] >= 9999) ? -1 : v[2];
}

void setServo(int angle) {
  servoAngle = constrain(angle, 0, 180);
  myServo.write(servoAngle);
}

// =====================================================================
//  LAPTOP API STATE
// =====================================================================
enum ApiState { API_IDLE, API_DRIVE, API_TURN, API_SCAN };
ApiState apiState = API_IDLE;
long apiId = 0;
String apiResult = "idle";            // idle | running | done | blocked | stuck | stopped | lost
unsigned long apiStart = 0, apiDuration = 0, apiElapsed = 0;
bool apiMotorsOn = false;
char apiDir = 'F';
int apiSpeed = 180;
int apiSafeCm = 18;
int apiLowCount = 0;
// while driving: servo sweeps 90 -> 72 -> 108 (a wider "bumper") and the back sensor is read too
const int SWEEP[4] = {90, 72, 108, -1};     // -1 = read the back sensor
int sweepI = 0;
// stall detection: if neither sensor's distance changes, the wheels are spinning against something
float stallF0 = -1, stallB0 = -1;
unsigned long stallT0 = 0;
const unsigned long STALL_MS = 1200;
unsigned long lastApiContact = 0;

const int MAX_SCAN = 16;
int   apiScanA[MAX_SCAN];
float apiScanD[MAX_SCAN];
int   apiScanN = 0, apiScanI = 0, apiScanPhase = 0;
unsigned long apiScanT = 0;
float apiScanBack = -1;

const char* stateName() {
  switch (apiState) {
    case API_DRIVE: return "drive";
    case API_TURN:  return "turn";
    case API_SCAN:  return "scan";
    default:        return "idle";
  }
}

void finishApi(const char* result, unsigned long elapsed) {
  stopMotors();
  apiResult = result;
  apiElapsed = elapsed;
  if (apiState == API_SCAN) setServo(SERVO_CENTER);
  apiState = API_IDLE;
}

// ---------- Manual scan (phone dashboard) ----------
float scanValue(float d) { return (d < 0) ? 400 : d; }

void runScan() {
  if (scanStep == 0) return;
  if (millis() - scanStepTime < 450) return;
  scanStepTime = millis();
  switch (scanStep) {
    case 1: setServo(SERVO_RIGHT); scanStep = 2; break;
    case 2: scanRight = readDistance(TRIG_F, ECHO_F);
            setServo(SERVO_CENTER); scanStep = 3; break;
    case 3: scanCenter = readDistance(TRIG_F, ECHO_F);
            setServo(SERVO_LEFT); scanStep = 4; break;
    case 4: {
      scanLeft = readDistance(TRIG_F, ECHO_F);
      setServo(SERVO_CENTER);
      float l = scanValue(scanLeft), c = scanValue(scanCenter), r = scanValue(scanRight);
      if (c >= STOP_DISTANCE && c >= l && c >= r) scanResult = "Path ahead is clear";
      else if (l < STOP_DISTANCE && r < STOP_DISTANCE && c < STOP_DISTANCE) scanResult = "Blocked - reverse";
      else if (l >= r) scanResult = "Turn LEFT";
      else scanResult = "Turn RIGHT";
      scanStep = 0;
      break;
    }
  }
}

// ---------- Manual web page (phone) ----------
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SafePath Robot</title>
<style>
body{font-family:Arial,sans-serif;text-align:center;background:#edf2f7;margin:0;padding:14px}
h1{color:#16324f;margin:6px 0}
.card{background:#fff;border-radius:12px;padding:12px;margin:10px auto;max-width:380px}
.row{display:flex;gap:10px;justify-content:center;max-width:380px;margin:10px auto}
.sensor{flex:1;background:#fff;border-radius:12px;padding:10px}
.val{font-size:28px;font-weight:bold;color:#1769aa}
.warn .val{color:#d62828}
button{width:100px;height:72px;margin:5px;border:none;border-radius:14px;background:#1769aa;color:#fff;font-size:17px;font-weight:bold;touch-action:none;user-select:none}
button:active{background:#0b416e}
.stop{background:#d62828}
.small{width:78px;height:44px;font-size:14px}
.scan{background:#2a9d8f;width:150px;height:48px}
input[type=range]{width:90%}
.scanvals{display:flex;justify-content:space-around;margin-top:8px}
</style>
</head>
<body>
<h1>SafePath Robot</h1>
<div class="row">
  <div class="sensor" id="fbox"><div>Front</div><div class="val" id="front">--</div><div>cm</div></div>
  <div class="sensor" id="bbox"><div>Back</div><div class="val" id="back">--</div><div>cm</div></div>
</div>
<div class="card" id="status">Connected</div>
<div>
  <div><button data-dir="F">&#9650;<br>Forward</button></div>
  <div>
    <button data-dir="L">&#9664;<br>Left</button>
    <button class="stop" id="stop">STOP</button>
    <button data-dir="R">&#9654;<br>Right</button>
  </div>
  <div><button data-dir="B">&#9660;<br>Reverse</button></div>
</div>
<div class="card">
  <div>Motor speed: <b id="spv">200</b> / 255</div>
  <input type="range" min="110" max="255" value="200" id="speed">
  <div>
    <button class="small" onclick="setSpeed(140)">Slow</button>
    <button class="small" onclick="setSpeed(200)">Medium</button>
    <button class="small" onclick="setSpeed(255)">Fast</button>
  </div>
</div>
<div class="card">
  <div>Front sensor direction: <b id="sv">90</b>&deg;</div>
  <input type="range" min="0" max="180" value="90" id="servo">
  <div>
    <button class="small" onclick="look('left')">Look L</button>
    <button class="small" onclick="look('center')">Center</button>
    <button class="small" onclick="look('right')">Look R</button>
  </div>
  <button class="scan" onclick="scan()">Scan Around</button>
  <div class="scanvals">
    <div>L<br><b id="sl">--</b></div><div>C<br><b id="sc">--</b></div><div>R<br><b id="sr">--</b></div>
  </div>
  <div style="margin-top:8px"><b id="advice">Not scanned yet</b></div>
</div>
<script>
let repeatTimer=null,currentDirection=null;
const st=document.getElementById('status');
const slider=document.getElementById('servo');
let dragging=false;
function sendCommand(dir){
  fetch('/move?dir='+dir,{cache:'no-store'})
   .then(r=>r.text()).then(t=>{st.textContent=(t==='BLOCKED')?'Obstacle! Movement blocked':(dir==='S'?'Stopped':'Moving: '+dir);})
   .catch(()=>{st.textContent='Connection lost - check robot';});
}
function releaseDirection(){
  if(repeatTimer!==null){clearInterval(repeatTimer);repeatTimer=null;}
  currentDirection=null; sendCommand('S');
}
document.querySelectorAll('[data-dir]').forEach(b=>{
  b.addEventListener('pointerdown',e=>{
    e.preventDefault(); releaseDirection();
    currentDirection=b.dataset.dir; sendCommand(currentDirection);
    repeatTimer=setInterval(()=>{if(currentDirection)sendCommand(currentDirection);},200);
  });
  b.addEventListener('pointerup',releaseDirection);
  b.addEventListener('pointercancel',releaseDirection);
  b.addEventListener('lostpointercapture',releaseDirection);
});
document.getElementById('stop').addEventListener('click',releaseDirection);
window.addEventListener('blur',releaseDirection);
document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseDirection();});
const spd=document.getElementById('speed');
function setSpeed(v){spd.value=v;document.getElementById('spv').textContent=v;
  fetch('/speed?val='+v,{cache:'no-store'}).catch(()=>{});}
spd.addEventListener('input',()=>{document.getElementById('spv').textContent=spd.value;});
spd.addEventListener('change',()=>setSpeed(spd.value));
function look(p){fetch('/servo?pos='+p,{cache:'no-store'}).catch(()=>{});}
slider.addEventListener('input',()=>{dragging=true;document.getElementById('sv').textContent=slider.value;});
slider.addEventListener('change',()=>{fetch('/servo?angle='+slider.value,{cache:'no-store'}).finally(()=>{dragging=false;});});
function scan(){st.textContent='Scanning...';fetch('/scan',{cache:'no-store'}).catch(()=>{});}
function fmt(v){return (v<0)?'--':v.toFixed(0);}
function show(id,box,v){
  document.getElementById(id).textContent=(v<0)?'--':v.toFixed(1);
  document.getElementById(box).className='sensor'+((v>0&&v<20)?' warn':'');
}
setInterval(()=>{
  fetch('/data',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    show('front','fbox',d.front); show('back','bbox',d.back);
    if(!dragging){slider.value=d.servo;document.getElementById('sv').textContent=d.servo;}
    document.getElementById('sl').textContent=fmt(d.sl);
    document.getElementById('sc').textContent=fmt(d.sc);
    document.getElementById('sr').textContent=fmt(d.sr);
    document.getElementById('advice').textContent=d.scanning?'Scanning...':d.advice;
  }).catch(()=>{});
},300);
</script>
</body>
</html>
)rawliteral";

// ---------- Manual handlers ----------
void handleRoot() { server.send(200, "text/html", webpage); }

void handleMove() {
  if (!server.hasArg("dir")) { stopMotors(); server.send(400, "text/plain", "Missing"); return; }
  String dir = server.arg("dir");
  lastCommandTime = millis();

  if (apiState != API_IDLE || (scanStep != 0 && dir != "S")) {
    if (apiState == API_IDLE) stopMotors();
    server.send(200, "text/plain", "BLOCKED"); return;
  }
  if (dir == "F") {
    if (frontBlocked()) { stopMotors(); server.send(200, "text/plain", "BLOCKED"); return; }
    forward();
  } else if (dir == "B") {
    if (backBlocked()) { stopMotors(); server.send(200, "text/plain", "BLOCKED"); return; }
    reverse();
  } else if (dir == "L") turnLeft();
  else if (dir == "R") turnRight();
  else stopMotors();
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("val")) {
    motorSpeed = constrain(server.arg("val").toInt(), MIN_SPEED, 255);
    if (apiState == API_IDLE) {
      if (currentDir == 'F' || currentDir == 'B') applySpeed(motorSpeed);
      else if (currentDir == 'L' || currentDir == 'R')
        applySpeed(max(MIN_SPEED, motorSpeed * TURN_PERCENT / 100));
    }
  }
  server.send(200, "text/plain", String(motorSpeed));
}

void handleServo() {
  if (scanStep == 0 && apiState == API_IDLE) {
    if (server.hasArg("pos")) {
      String p = server.arg("pos");
      if (p == "left") setServo(SERVO_LEFT);
      else if (p == "right") setServo(SERVO_RIGHT);
      else setServo(SERVO_CENTER);
    } else if (server.hasArg("angle")) {
      setServo(server.arg("angle").toInt());
    }
  }
  server.send(200, "text/plain", String(servoAngle));
}

void handleScan() {
  if (apiState == API_IDLE) {
    stopMotors();
    if (scanStep == 0) { scanStep = 1; scanStepTime = 0; }
  }
  server.send(200, "text/plain", "OK");
}

void handleData() {
  String json = "{\"front\":" + String(distFront, 1) +
                ",\"back\":"  + String(distBack, 1) +
                ",\"servo\":" + String(servoAngle) +
                ",\"sl\":"    + String(scanLeft, 1) +
                ",\"sc\":"    + String(scanCenter, 1) +
                ",\"sr\":"    + String(scanRight, 1) +
                ",\"scanning\":" + String(scanStep != 0 ? "true" : "false") +
                ",\"advice\":\"" + scanResult + "\"}";
  server.send(200, "application/json", json);
}

// =====================================================================
//  LAPTOP API HANDLERS
// =====================================================================
void cors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Cache-Control", "no-store");
}
void sendJson(const String& s) { cors(); server.send(200, "application/json", s); }

String statusJson() {
  String s = "{\"id\":" + String(apiId) +
             ",\"state\":\"" + stateName() + "\"" +
             ",\"result\":\"" + apiResult + "\"" +
             ",\"elapsed\":" + String(apiElapsed) +
             ",\"front\":" + String(distFront, 1) +
             ",\"back\":" + String(distBack, 1) +
             ",\"servo\":" + String(servoAngle) +
             ",\"scan\":[";
  for (int i = 0; i < apiScanN; i++) {
    if (i) s += ",";
    s += "[" + String(apiScanA[i]) + "," + String(apiScanD[i], 1) + "]";
  }
  s += "],\"sback\":" + String(apiScanBack, 1) +
       ",\"uptime\":" + String(millis()) + "}";
  return s;
}

// Returns true if the request was already handled (duplicate id or busy)
bool apiPrecheck() {
  lastApiContact = millis();
  long id = server.hasArg("id") ? server.arg("id").toInt() : 0;
  if (id != 0 && id == apiId) { sendJson(statusJson()); return true; }   // retry of same command
  if (apiState != API_IDLE || scanStep != 0) { sendJson("{\"err\":\"busy\"}"); return true; }
  apiId = id;
  return false;
}

void handleApiPing() {
  lastApiContact = millis();
  sendJson("{\"ok\":true,\"name\":\"SafePath\",\"fw\":\"auto-1.0\"}");
}

void handleApiStatus() {
  lastApiContact = millis();
  sendJson(statusJson());
}

void handleApiStop() {
  lastApiContact = millis();
  if (apiState != API_IDLE) {
    unsigned long el = ((apiState == API_DRIVE || apiState == API_TURN) && apiMotorsOn) ? millis() - apiStart : 0;
    finishApi("stopped", el);
  }
  scanStep = 0;
  stopMotors();
  sendJson(statusJson());
}

void handleApiDrive() {
  if (apiPrecheck()) return;
  apiDir = (server.arg("dir") == "B") ? 'B' : 'F';
  apiDuration = constrain(server.arg("ms").toInt(), 0, 4000);
  apiSpeed = server.hasArg("spd") ? constrain(server.arg("spd").toInt(), MIN_SPEED, 255) : motorSpeed;
  apiSafeCm = server.hasArg("safe") ? constrain(server.arg("safe").toInt(), 5, 100) : STOP_DISTANCE;
  apiResult = "running"; apiElapsed = 0; apiLowCount = 0; apiMotorsOn = false;
  sweepI = 0; stallF0 = -1; stallB0 = -1;
  unsigned long settle = 0;
  if (apiDir == 'F' && abs(servoAngle - SERVO_CENTER) > 3) { setServo(SERVO_CENTER); settle = 400; }
  apiStart = millis() + settle;
  apiState = API_DRIVE;
  sendJson(statusJson());
}

void handleApiTurn() {
  if (apiPrecheck()) return;
  apiDir = (server.arg("dir") == "L") ? 'L' : 'R';
  apiDuration = constrain(server.arg("ms").toInt(), 0, 3000);
  apiSpeed = server.hasArg("spd") ? constrain(server.arg("spd").toInt(), MIN_SPEED, 255) : motorSpeed;
  apiResult = "running"; apiElapsed = 0; apiMotorsOn = false;
  apiStart = millis();
  apiState = API_TURN;
  sendJson(statusJson());
}

void handleApiScan() {
  if (apiPrecheck()) return;
  String a = server.arg("a");
  apiScanN = 0;
  int start = 0;
  while (start < (int)a.length() && apiScanN < MAX_SCAN) {
    int c = a.indexOf(',', start);
    if (c < 0) c = a.length();
    apiScanA[apiScanN++] = constrain(a.substring(start, c).toInt(), 0, 180);
    start = c + 1;
  }
  if (apiScanN == 0) {
    int def[5] = {20, 55, 90, 125, 160};
    for (int i = 0; i < 5; i++) apiScanA[i] = def[i];
    apiScanN = 5;
  }
  for (int i = 0; i < apiScanN; i++) apiScanD[i] = -1;
  apiScanBack = -1;
  apiScanI = 0;
  stopMotors();
  setServo(apiScanA[0]);
  apiScanT = millis();
  apiScanPhase = 1;
  apiResult = "running"; apiElapsed = 0;
  apiState = API_SCAN;
  sendJson(statusJson());
}

void handleNotFound() {
  if (server.method() == HTTP_OPTIONS) {          // browser pre-flight check
    cors();
    server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "*");
    server.send(204);
    return;
  }
  server.send(404, "text/plain", "Not found");
}

// ---------- Runs the active laptop command (non-blocking) ----------
void updateApi() {
  unsigned long now = millis();

  // Safety: laptop went silent in the middle of a move -> stop
  if ((apiState == API_DRIVE || apiState == API_TURN) && now - lastApiContact > LAPTOP_TIMEOUT) {
    finishApi("lost", apiMotorsOn ? now - apiStart : 0);
    return;
  }

  if (apiState == API_SCAN) {
    unsigned long settle = (apiScanI == 0) ? 450 : 280;
    if (apiScanPhase == 1 && now - apiScanT >= settle) {
      apiScanD[apiScanI] = medianRead(TRIG_F, ECHO_F);
      apiScanI++;
      if (apiScanI < apiScanN) {
        setServo(apiScanA[apiScanI]);
      } else {
        apiScanBack = medianRead(TRIG_B, ECHO_B);
        distBack = apiScanBack;
        setServo(SERVO_CENTER);
        apiScanPhase = 2;
      }
      apiScanT = millis();
    } else if (apiScanPhase == 2 && now - apiScanT >= 350) {
      distFront = medianRead(TRIG_F, ECHO_F);
      finishApi("done", 0);
    }
    return;
  }

  // DRIVE / TURN
  if ((long)(now - apiStart) < 0) return;          // waiting for servo to centre

  if (!apiMotorsOn) {
    if (apiState == API_DRIVE) {
      float d = (apiDir == 'F') ? readDistance(TRIG_F, ECHO_F) : readDistance(TRIG_B, ECHO_B);
      if (apiDir == 'F') distFront = d; else distBack = d;
      if (d > 0 && d < apiSafeCm) { finishApi("blocked", 0); return; }
      if (apiDir == 'F') forward(); else reverse();
    } else {
      if (apiDir == 'L') turnLeft(); else turnRight();
    }
    applySpeed(MIN_SPEED);                         // soft start - ramps up below
    apiMotorsOn = true;
    apiStart = millis();
    lastSensorRead = 0;
    stallT0 = millis();
    return;
  }

  unsigned long el = millis() - apiStart;

  // soft start: ramp to the requested speed over 300 ms (less jerk, less wheel slip)
  if (el < 300) applySpeed(MIN_SPEED + (apiSpeed - MIN_SPEED) * (long)el / 300);
  else applySpeed(apiSpeed);

  // Reflex: emergency brake while driving
  if (apiState == API_DRIVE && apiDir == 'F' && millis() - lastSensorRead >= 70) {
    lastSensorRead = millis();
    int a = SWEEP[sweepI];
    if (a < 0) {
      distBack = readDistance(TRIG_B, ECHO_B);
    } else {
      float d = readDistance(TRIG_F, ECHO_F);
      if (a == 90) distFront = d;
      // side beams: only brake for things roughly in the robot's path
      float limit = (a == 90) ? apiSafeCm : apiSafeCm * 0.85;
      if (d > 0 && d < limit) {
        if (++apiLowCount >= 2) { setServo(SERVO_CENTER); finishApi("blocked", millis() - apiStart); return; }
      } else if (a == 90) {
        apiLowCount = 0;
      }
    }
    sweepI = (sweepI + 1) % 4;
    if (SWEEP[sweepI] >= 0) setServo(SWEEP[sweepI]);
  }
  if (apiState == API_DRIVE && apiDir == 'B' && millis() - lastSensorRead >= 40) {
    lastSensorRead = millis();
    sweepI = (sweepI + 1) % 4;
    float d;
    if (sweepI == 0) { d = readDistance(TRIG_F, ECHO_F); distFront = d; }   // occasionally, for stall check
    else {
      d = readDistance(TRIG_B, ECHO_B); distBack = d;
      if (d > 0 && d < apiSafeCm) {
        if (++apiLowCount >= 2) { finishApi("blocked", millis() - apiStart); return; }
      } else apiLowCount = 0;
    }
  }

  // Stall: wheels turning but neither the front nor the back distance changes -> stuck
  if (apiState == API_DRIVE && millis() - stallT0 >= STALL_MS) {
    bool fOk = distFront > 0 && distFront < 150 && stallF0 > 0;
    bool bOk = distBack > 0 && distBack < 150 && stallB0 > 0;
    bool fSame = fOk && fabs(distFront - stallF0) < 2.5;
    bool bSame = bOk && fabs(distBack - stallB0) < 2.5;
    if ((fOk || bOk) && (!fOk || fSame) && (!bOk || bSame)) {
      setServo(SERVO_CENTER);
      finishApi("stuck", millis() - apiStart);
      return;
    }
    stallF0 = distFront; stallB0 = distBack; stallT0 = millis();
  }

  if (el >= apiDuration) { if (apiDir == 'F') setServo(SERVO_CENTER); finishApi("done", apiDuration); }
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  pwmA.attachPin(ENA, 1000, 8);
  pwmB.attachPin(ENB, 1000, 8);
  stopMotors();

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  digitalWrite(TRIG_F, LOW); digitalWrite(TRIG_B, LOW);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  setServo(SERVO_CENTER);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);                 // faster, steadier replies

  // phone (manual)
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/speed", handleSpeed);
  server.on("/servo", handleServo);
  server.on("/scan", handleScan);
  server.on("/data", handleData);
  // laptop (auto)
  server.on("/api/ping",   HTTP_GET, handleApiPing);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/stop",   HTTP_GET, handleApiStop);
  server.on("/api/drive",  HTTP_GET, handleApiDrive);
  server.on("/api/turn",   HTTP_GET, handleApiTurn);
  server.on("/api/scan",   HTTP_GET, handleApiScan);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.print("SafePath AUTO ready. Dashboard host: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  if (apiState != API_IDLE) { updateApi(); return; }

  // ----- manual mode / idle -----
  if (scanStep != 0) {
    runScan();
  } else if (millis() - lastSensorRead > 50) {
    lastSensorRead = millis();
    if (readFrontNext) distFront = readDistance(TRIG_F, ECHO_F);
    else               distBack  = readDistance(TRIG_B, ECHO_B);
    readFrontNext = !readFrontNext;
  }

  if ((currentDir == 'F' && frontBlocked()) || (currentDir == 'B' && backBlocked())) {
    stopMotors();
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    stopMotors();
  }
}
