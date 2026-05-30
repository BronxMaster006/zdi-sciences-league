#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Preferences.h>

// ================================================================
// ESP32 N1 - 2-Achs-Magnetarm fuer zdi/FoodConnect
// Kompiliert mit Arduino-ESP32. Keine externen Libraries noetig.
// ================================================================

// ------------------------- Feste Konfiguration ------------------
const char *WIFI_SSID = "FoodConnect-System";
const char *WIFI_PASS = "12345678";
const char *N2_BASE_URL = "http://192.168.4.1";

const bool ARM2_VIRTUAL_MIRROR = true;   // Feste virtuelle Spiegelung fuer Arm 2, nicht per Website aenderbar

const uint8_t PIN_M1_STEP = 25;
const uint8_t PIN_M1_DIR  = 26;
const uint8_t PIN_M2_STEP = 18;
const uint8_t PIN_M2_DIR  = 19;
const uint8_t PIN_A1_MIN  = 32;
const uint8_t PIN_A1_MAX  = 33;

const uint8_t ARM1_SDA = 21;
const uint8_t ARM1_SCL = 22;
const uint8_t ARM2_SDA = 16;
const uint8_t ARM2_SCL = 17;
const uint8_t AS5600_ADDR = 0x36;

const float ARM1_LEN_CM = 20.0f;
const float ARM2_LEN_CM = 20.0f;
const float DEFAULT_PLATE_W_CM = 80.0f;
const float DEFAULT_PLATE_H_CM = 60.0f;
const float DEFAULT_BASE_X_CM = 40.0f;
const float DEFAULT_BASE_Y_CM = 30.0f;

const uint16_t STEP_PULSE_US = 700;
const uint16_t STEP_GAP_US = 700;
const uint16_t STEPS_PER_DEGREE = 8;       // Bei Bedarf mechanisch anpassen
const uint32_t MOVE_TIMEOUT_MS = 20000;
const uint32_t HTTP_TIMEOUT_MS = 1200;
const float ANGLE_TOL_DEG = 1.2f;
const float BARRIER_MARGIN_CM = 1.5f;
const uint8_t POSITION_COUNT = 8;
const uint8_t BARRIER_COUNT = 8;

// ------------------------- Datenstrukturen ----------------------
struct EncoderData {
  bool found;
  bool magnetDetected;
  bool magnetTooWeak;
  bool magnetTooStrong;
  uint16_t raw;
  float rawDeg;
  float calibratedDeg;
};

struct StoredPosition {
  bool valid;
  float arm1Deg;
  float arm2Deg;
};

struct BarrierRect {
  bool enabled;
  float x;
  float y;
  float w;
  float h;
};

struct IkSolution {
  bool valid;
  float arm1Deg;
  float arm2Deg;
  float score;
};

// ------------------------- Globale Objekte ----------------------
WebServer server(80);
Preferences prefs;
TwoWire WireArm2 = TwoWire(1);

EncoderData enc1;
EncoderData enc2;
StoredPosition positions[POSITION_COUNT];
BarrierRect barriers[BARRIER_COUNT];

uint16_t arm1RawZero = 0;
uint16_t arm1Raw270 = 3072;
uint16_t arm2RawZero = 0;
float arm2SoftMinDeg = 0.0f;
float arm2SoftMaxDeg = 180.0f;
bool motor1Reverse = false;
bool motor2Reverse = false;
bool stepperPowerOn = false;
bool n2Reachable = false;
float plateW = DEFAULT_PLATE_W_CM;
float plateH = DEFAULT_PLATE_H_CM;
float baseX = DEFAULT_BASE_X_CM;
float baseY = DEFAULT_BASE_Y_CM;
String lastMessage = "Bereit";

// ------------------------- Hilfsfunktionen ----------------------
float normalize360(float a) {
  while (a < 0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

float angleDiffSigned(float target, float current) {
  float d = fmodf((target - current + 540.0f), 360.0f) - 180.0f;
  return d;
}

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

String yn(bool v) {
  return v ? "true" : "false";
}

String jsonEscape(const String &s) {
  String out;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else out += c;
  }
  return out;
}

bool limitMinPressed() { return digitalRead(PIN_A1_MIN) == LOW; }
bool limitMaxPressed() { return digitalRead(PIN_A1_MAX) == LOW; }

float rawDeltaDeg(uint16_t raw, uint16_t zero) {
  int32_t d = (int32_t)raw - (int32_t)zero;
  while (d < 0) d += 4096;
  while (d >= 4096) d -= 4096;
  return (float)d * 360.0f / 4096.0f;
}

float arm1CalibratedFromRaw(uint16_t raw) {
  float span = rawDeltaDeg(arm1Raw270, arm1RawZero);
  if (span < 5.0f) span = 270.0f;
  float d = rawDeltaDeg(raw, arm1RawZero);
  return clampFloat(d * 270.0f / span, 0.0f, 270.0f);
}

float arm2CalibratedFromRaw(uint16_t raw) {
  return normalize360(rawDeltaDeg(raw, arm2RawZero));
}

float arm2ToVirtual(float physicalDeg) {
  return ARM2_VIRTUAL_MIRROR ? -physicalDeg : physicalDeg;
}

float arm2FromVirtual(float virtualDeg) {
  return normalize360(ARM2_VIRTUAL_MIRROR ? -virtualDeg : virtualDeg);
}

bool angleInsideSoftLimits(float deg) {
  deg = normalize360(deg);
  float minD = normalize360(arm2SoftMinDeg);
  float maxD = normalize360(arm2SoftMaxDeg);
  if (fabsf(angleDiffSigned(maxD, minD)) < 0.5f) return true;
  if (minD <= maxD) return deg >= minD && deg <= maxD;
  return deg >= minD || deg <= maxD;
}

bool readAS5600(TwoWire &bus, EncoderData &out, bool arm1) {
  out.found = false;
  out.magnetDetected = false;
  out.magnetTooWeak = false;
  out.magnetTooStrong = false;
  out.raw = 0;
  out.rawDeg = 0.0f;
  out.calibratedDeg = 0.0f;

  bus.beginTransmission(AS5600_ADDR);
  if (bus.endTransmission() != 0) return false;
  out.found = true;

  bus.beginTransmission(AS5600_ADDR);
  bus.write(0x0B);
  if (bus.endTransmission(false) == 0 && bus.requestFrom((int)AS5600_ADDR, 1) == 1) {
    uint8_t status = bus.read();
    out.magnetDetected = (status & 0x20) != 0;
    out.magnetTooWeak = (status & 0x10) != 0;
    out.magnetTooStrong = (status & 0x08) != 0;
  }

  bus.beginTransmission(AS5600_ADDR);
  bus.write(0x0C);
  if (bus.endTransmission(false) != 0) return true;
  if (bus.requestFrom((int)AS5600_ADDR, 2) == 2) {
    uint8_t hi = bus.read();
    uint8_t lo = bus.read();
    out.raw = (((uint16_t)hi << 8) | lo) & 0x0FFF;
    out.rawDeg = (float)out.raw * 360.0f / 4096.0f;
    out.calibratedDeg = arm1 ? arm1CalibratedFromRaw(out.raw) : arm2CalibratedFromRaw(out.raw);
  }
  return true;
}

void updateEncoders() {
  readAS5600(Wire, enc1, true);
  readAS5600(WireArm2, enc2, false);
}

bool httpGetN2(const String &path) {
  if (WiFi.status() != WL_CONNECTED) {
    n2Reachable = false;
    return false;
  }
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  String url = String(N2_BASE_URL) + path;
  if (!http.begin(url)) {
    n2Reachable = false;
    return false;
  }
  int code = http.GET();
  http.end();
  n2Reachable = (code > 0 && code < 500);
  return code >= 200 && code < 300;
}

bool requestStepperPower(bool on) {
  bool ok = httpGetN2(String("/api/stepperpower?state=") + (on ? "1" : "0"));
  if (ok) stepperPowerOn = on;
  lastMessage = ok ? (on ? "Stepper-Strom freigegeben" : "Stepper-Strom abgeschaltet") : "N2/Stepperfreigabe nicht erreichbar";
  return ok;
}

void emergencyStop() {
  httpGetN2("/api/emergency");
  stepperPowerOn = false;
  lastMessage = "Not-Aus gesendet";
}

bool sensorsReadyForMotion() {
  updateEncoders();
  if (!enc1.found || !enc2.found) {
    lastMessage = "Bewegung gesperrt: AS5600 nicht gefunden";
    return false;
  }
  if (!enc1.magnetDetected || !enc2.magnetDetected) {
    lastMessage = "Bewegung gesperrt: Magnet nicht erkannt";
    return false;
  }
  return true;
}

void setDir(uint8_t dirPin, bool forward, bool reversed) {
  digitalWrite(dirPin, (forward ^ reversed) ? HIGH : LOW);
}

bool singleStep(uint8_t stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_GAP_US);
  server.handleClient();
  return true;
}

bool moveMotorSteps(uint8_t motor, long steps) {
  if (steps == 0) return true;
  if (!sensorsReadyForMotion()) return false;
  if (!requestStepperPower(true)) return false;

  uint8_t stepPin = motor == 1 ? PIN_M1_STEP : PIN_M2_STEP;
  uint8_t dirPin = motor == 1 ? PIN_M1_DIR : PIN_M2_DIR;
  bool reversed = motor == 1 ? motor1Reverse : motor2Reverse;
  bool forward = steps > 0;
  long count = labs(steps);
  setDir(dirPin, forward, reversed);

  uint32_t start = millis();
  for (long i = 0; i < count; i++) {
    if (millis() - start > MOVE_TIMEOUT_MS) {
      lastMessage = "Motor-Test abgebrochen: Timeout";
      requestStepperPower(false);
      return false;
    }
    if (motor == 1 && ((forward && limitMaxPressed()) || (!forward && limitMinPressed()))) {
      lastMessage = "Motor 1 gestoppt: Endschalter erreicht";
      requestStepperPower(false);
      return false;
    }
    if (motor == 2) {
      updateEncoders();
      if (enc2.found && enc2.magnetDetected && !angleInsideSoftLimits(enc2.calibratedDeg)) {
        lastMessage = "Motor 2 gestoppt: Softwaregrenze erreicht";
        requestStepperPower(false);
        return false;
      }
    }
    singleStep(stepPin);
  }
  updateEncoders();
  lastMessage = "Motor-Test fertig";
  requestStepperPower(false);
  return true;
}

bool moveToAngles(float targetArm1, float targetArm2) {
  targetArm1 = clampFloat(targetArm1, 0.0f, 270.0f);
  targetArm2 = normalize360(targetArm2);
  if (!angleInsideSoftLimits(targetArm2)) {
    lastMessage = "Ziel abgelehnt: Arm 2 ausserhalb Softwaregrenzen";
    return false;
  }
  if (!sensorsReadyForMotion()) return false;
  if (!requestStepperPower(true)) return false;

  uint32_t start = millis();
  while (millis() - start <= MOVE_TIMEOUT_MS) {
    updateEncoders();
    if (!enc1.found || !enc2.found || !enc1.magnetDetected || !enc2.magnetDetected) {
      lastMessage = "Fahrt abgebrochen: Sensor/Magnet verloren";
      requestStepperPower(false);
      return false;
    }

    float d1 = targetArm1 - enc1.calibratedDeg;
    float d2 = angleDiffSigned(targetArm2, enc2.calibratedDeg); // Wichtig: sauber ueber 0/360
    bool done1 = fabsf(d1) <= ANGLE_TOL_DEG;
    bool done2 = fabsf(d2) <= ANGLE_TOL_DEG;
    if (done1 && done2) {
      lastMessage = "Ziel erreicht";
      requestStepperPower(false);
      return true;
    }

    if (!done1) {
      bool fwd1 = d1 > 0;
      if ((fwd1 && limitMaxPressed()) || (!fwd1 && limitMinPressed())) {
        lastMessage = "Arm 1 gestoppt: Endschalter";
        requestStepperPower(false);
        return false;
      }
      setDir(PIN_M1_DIR, fwd1, motor1Reverse);
      singleStep(PIN_M1_STEP);
    }

    if (!done2) {
      if (!angleInsideSoftLimits(enc2.calibratedDeg)) {
        lastMessage = "Arm 2 gestoppt: Softwaregrenze";
        requestStepperPower(false);
        return false;
      }
      bool fwd2 = d2 > 0;
      setDir(PIN_M2_DIR, fwd2, motor2Reverse);
      singleStep(PIN_M2_STEP);
    }
  }

  lastMessage = "Fahrt abgebrochen: Timeout";
  requestStepperPower(false);
  return false;
}

// ------------------------- Geometrie/Sicherheit -----------------
void forwardKinematics(float a1Deg, float a2PhysDeg, float &j1x, float &j1y, float &tipX, float &tipY) {
  float a1 = radians(a1Deg);
  float a2 = radians(a1Deg + arm2ToVirtual(a2PhysDeg));
  j1x = baseX + cosf(a1) * ARM1_LEN_CM;
  j1y = baseY + sinf(a1) * ARM1_LEN_CM;
  tipX = j1x + cosf(a2) * ARM2_LEN_CM;
  tipY = j1y + sinf(a2) * ARM2_LEN_CM;
}

float distPointSegment(float px, float py, float ax, float ay, float bx, float by) {
  float vx = bx - ax;
  float vy = by - ay;
  float wx = px - ax;
  float wy = py - ay;
  float len2 = vx * vx + vy * vy;
  float t = len2 > 0.0001f ? (wx * vx + wy * vy) / len2 : 0.0f;
  t = clampFloat(t, 0.0f, 1.0f);
  float cx = ax + t * vx;
  float cy = ay + t * vy;
  float dx = px - cx;
  float dy = py - cy;
  return sqrtf(dx * dx + dy * dy);
}

bool segmentHitsRect(float ax, float ay, float bx, float by, const BarrierRect &r, float margin) {
  if (!r.enabled) return false;
  float minX = r.x - margin;
  float minY = r.y - margin;
  float maxX = r.x + r.w + margin;
  float maxY = r.y + r.h + margin;
  if ((ax >= minX && ax <= maxX && ay >= minY && ay <= maxY) || (bx >= minX && bx <= maxX && by >= minY && by <= maxY)) return true;
  float corners[4][2] = {{minX, minY}, {maxX, minY}, {maxX, maxY}, {minX, maxY}};
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t j = (i + 1) % 4;
    float x1 = corners[i][0], y1 = corners[i][1], x2 = corners[j][0], y2 = corners[j][1];
    float den = (bx - ax) * (y2 - y1) - (by - ay) * (x2 - x1);
    if (fabsf(den) < 0.0001f) continue;
    float ua = ((x2 - x1) * (ay - y1) - (y2 - y1) * (ax - x1)) / den;
    float ub = ((bx - ax) * (ay - y1) - (by - ay) * (ax - x1)) / den;
    if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f) return true;
  }
  return false;
}

bool poseHitsBarrier(float a1Deg, float a2PhysDeg) {
  float jx, jy, tx, ty;
  forwardKinematics(a1Deg, a2PhysDeg, jx, jy, tx, ty);
  for (uint8_t i = 0; i < BARRIER_COUNT; i++) {
    if (segmentHitsRect(baseX, baseY, jx, jy, barriers[i], BARRIER_MARGIN_CM)) return true;
    if (segmentHitsRect(jx, jy, tx, ty, barriers[i], BARRIER_MARGIN_CM)) return true;
  }
  return false;
}

bool pathIsSafe(float startA1, float startA2, float targetA1, float targetA2) {
  const uint8_t samples = 30;
  for (uint8_t i = 0; i <= samples; i++) {
    float t = (float)i / (float)samples;
    float a1 = startA1 + (targetA1 - startA1) * t;
    float d2 = angleDiffSigned(targetA2, startA2);
    float a2 = normalize360(startA2 + d2 * t);
    if (a1 < 0.0f || a1 > 270.0f) return false;
    if (!angleInsideSoftLimits(a2)) return false;
    if (poseHitsBarrier(a1, a2)) return false;
  }
  return true;
}

IkSolution makeIkSolution(float x, float y, int elbowSign) {
  IkSolution s;
  s.valid = false;
  s.arm1Deg = 0.0f;
  s.arm2Deg = 0.0f;
  s.score = 999999.0f;

  float dx = x - baseX;
  float dy = y - baseY;
  float r2 = dx * dx + dy * dy;
  float c2 = (r2 - ARM1_LEN_CM * ARM1_LEN_CM - ARM2_LEN_CM * ARM2_LEN_CM) / (2.0f * ARM1_LEN_CM * ARM2_LEN_CM);
  if (c2 < -1.0f || c2 > 1.0f) return s;
  c2 = clampFloat(c2, -1.0f, 1.0f);
  float v2 = elbowSign * acosf(c2);
  float v1 = atan2f(dy, dx) - atan2f(ARM2_LEN_CM * sinf(v2), ARM1_LEN_CM + ARM2_LEN_CM * cosf(v2));
  float a1 = normalize360(degrees(v1));
  float a2Phys = arm2FromVirtual(degrees(v2));

  if (a1 > 270.0f) return s;
  if (!angleInsideSoftLimits(a2Phys)) return s;
  if (poseHitsBarrier(a1, a2Phys)) return s;
  s.valid = true;
  s.arm1Deg = a1;
  s.arm2Deg = a2Phys;
  s.score = fabsf(a1 - enc1.calibratedDeg) + fabsf(angleDiffSigned(a2Phys, enc2.calibratedDeg));
  return s;
}

bool smartMoveTo(float x, float y) {
  updateEncoders();
  if (!sensorsReadyForMotion()) return false;
  if (x < 0.0f || y < 0.0f || x > plateW || y > plateH) {
    lastMessage = "Ziel ausserhalb der Platte";
    return false;
  }
  IkSolution a = makeIkSolution(x, y, 1);
  IkSolution b = makeIkSolution(x, y, -1);
  if (a.valid && !pathIsSafe(enc1.calibratedDeg, enc2.calibratedDeg, a.arm1Deg, a.arm2Deg)) a.valid = false;
  if (b.valid && !pathIsSafe(enc1.calibratedDeg, enc2.calibratedDeg, b.arm1Deg, b.arm2Deg)) b.valid = false;

  IkSolution chosen;
  chosen.valid = false;
  if (a.valid && b.valid) chosen = (a.score <= b.score) ? a : b;
  else if (a.valid) chosen = a;
  else if (b.valid) chosen = b;
  else {
    lastMessage = "Kein sicherer IK-Pfad gefunden";
    return false;
  }
  return moveToAngles(chosen.arm1Deg, chosen.arm2Deg);
}

// ------------------------- NVS -------------------------------
void loadSettings() {
  prefs.begin("magarm", false);
  arm1RawZero = prefs.getUShort("a1z", 0);
  arm1Raw270 = prefs.getUShort("a1270", 3072);
  arm2RawZero = prefs.getUShort("a2z", 0);
  arm2SoftMinDeg = prefs.getFloat("a2min", 0.0f);
  arm2SoftMaxDeg = prefs.getFloat("a2max", 180.0f);
  motor1Reverse = prefs.getBool("m1rev", false);
  motor2Reverse = prefs.getBool("m2rev", false);
  plateW = prefs.getFloat("plateW", DEFAULT_PLATE_W_CM);
  plateH = prefs.getFloat("plateH", DEFAULT_PLATE_H_CM);
  baseX = prefs.getFloat("baseX", DEFAULT_BASE_X_CM);
  baseY = prefs.getFloat("baseY", DEFAULT_BASE_Y_CM);
  for (uint8_t i = 0; i < POSITION_COUNT; i++) {
    String p = "p" + String(i);
    positions[i].valid = prefs.getBool((p + "v").c_str(), false);
    positions[i].arm1Deg = prefs.getFloat((p + "a1").c_str(), 0.0f);
    positions[i].arm2Deg = prefs.getFloat((p + "a2").c_str(), 0.0f);
  }
  for (uint8_t i = 0; i < BARRIER_COUNT; i++) {
    String p = "b" + String(i);
    barriers[i].enabled = prefs.getBool((p + "e").c_str(), false);
    barriers[i].x = prefs.getFloat((p + "x").c_str(), 0.0f);
    barriers[i].y = prefs.getFloat((p + "y").c_str(), 0.0f);
    barriers[i].w = prefs.getFloat((p + "w").c_str(), 0.0f);
    barriers[i].h = prefs.getFloat((p + "h").c_str(), 0.0f);
  }
}

void savePositions() {
  for (uint8_t i = 0; i < POSITION_COUNT; i++) {
    String p = "p" + String(i);
    prefs.putBool((p + "v").c_str(), positions[i].valid);
    prefs.putFloat((p + "a1").c_str(), positions[i].arm1Deg);
    prefs.putFloat((p + "a2").c_str(), positions[i].arm2Deg);
  }
}

void saveBarriers() {
  for (uint8_t i = 0; i < BARRIER_COUNT; i++) {
    String p = "b" + String(i);
    prefs.putBool((p + "e").c_str(), barriers[i].enabled);
    prefs.putFloat((p + "x").c_str(), barriers[i].x);
    prefs.putFloat((p + "y").c_str(), barriers[i].y);
    prefs.putFloat((p + "w").c_str(), barriers[i].w);
    prefs.putFloat((p + "h").c_str(), barriers[i].h);
  }
}

// ------------------------- Web-App ----------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FoodConnect Magnetarm N1</title><style>
:root{--bg:#101827;--card:#182338;--txt:#eaf0ff;--mut:#8fa1c7;--ok:#2fe39b;--bad:#ff5d73;--acc:#6ca6ff;--warn:#ffd166}*{box-sizing:border-box}body{margin:0;font-family:system-ui,Segoe UI,Arial;background:linear-gradient(135deg,#0b1020,#17243b);color:var(--txt)}header{padding:22px 28px;border-bottom:1px solid #26344f;background:#111a2cdd;position:sticky;top:0;z-index:3}h1{margin:0;font-size:24px}.wrap{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;padding:16px}.card{background:var(--card);border:1px solid #2a3a5e;border-radius:18px;padding:16px;box-shadow:0 12px 35px #0005}h2{margin:0 0 12px;font-size:18px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.item{background:#0f1728;border-radius:12px;padding:9px}.label{color:var(--mut);font-size:12px}.val{font-weight:700}.ok{color:var(--ok)}.bad{color:var(--bad)}button,input{border:0;border-radius:11px;padding:10px 12px;margin:4px;background:#263a60;color:var(--txt);font-weight:650}button{cursor:pointer}button:hover{background:#34507f}.danger{background:#7d1730}.primary{background:#1f5fbf}.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center}input{background:#0f1728;border:1px solid #35486f;max-width:96px}.msg{color:var(--warn);min-height:24px}canvas{width:100%;max-width:820px;background:#0b1220;border:1px solid #40547a;border-radius:14px;touch-action:none}.wide{grid-column:1/-1}.small{font-size:12px;color:var(--mut)}
</style></head><body><header><h1>FoodConnect Magnetarm N1</h1><div class="small">ESP32 N1 · AS5600 · Stepper · virtueller Raum</div></header>
<div class="wrap">
<section class="card"><h2>Status</h2><div id="status" class="grid"></div><p class="msg" id="msg"></p><button class="danger" onclick="api('/api/emergency')">NOT-AUS</button></section>
<section class="card"><h2>Kalibrierung</h2><div class="row"><button onclick="api('/api/calibrate?cmd=a1zero')">Arm 1 = 0°</button><button onclick="api('/api/calibrate?cmd=a1270')">Arm 1 = 270°</button><button onclick="api('/api/calibrate?cmd=a2zero')">Arm 2 = 0°</button><button onclick="api('/api/calibrate?cmd=a2min')">Arm 2 MIN</button><button onclick="api('/api/calibrate?cmd=a2max')">Arm 2 MAX</button></div></section>
<section class="card"><h2>Motor-Test</h2><div id="motors"></div><button onclick="api('/api/reverse?motor=1')">Motor 1 Richtung umschalten</button><button onclick="api('/api/reverse?motor=2')">Motor 2 Richtung umschalten</button></section>
<section class="card"><h2>Positionen</h2><div id="positions"></div></section>
<section class="card wide"><h2>Virtueller Raum</h2><div class="row">Platte <input id="pw" type="number" step="1"> × <input id="ph" type="number" step="1"> cm Basis <input id="bx" type="number" step="1"> / <input id="by" type="number" step="1"> cm <button onclick="saveRoom()">Raum speichern</button></div><canvas id="map" width="900" height="620"></canvas><div class="row"><button onclick="barrierMode=!barrierMode;info()">Barriere zeichnen an/aus</button><button onclick="clearBarriers()">Barrieren löschen</button><span id="hint" class="small"></span></div></section>
</div>
<script>
let state=null,barrierMode=false,drawStart=null;const map=document.getElementById('map'),ctx=map.getContext('2d');
async function api(u){try{let r=await fetch(u);let t=await r.text();document.getElementById('msg').textContent=t;await load();}catch(e){document.getElementById('msg').textContent=e}}
function pill(v){return v?'<span class="ok">JA</span>':'<span class="bad">NEIN</span>'}
function item(k,v){return `<div class=item><div class=label>${k}</div><div class=val>${v}</div></div>`}
async function load(){let r=await fetch('/api/status');state=await r.json();document.getElementById('msg').textContent=state.message;renderStatus();renderPositions();draw();}
function renderStatus(){let s=state;document.getElementById('status').innerHTML=item('WLAN',pill(s.wifi))+item('N2 erreichbar',pill(s.n2))+item('Stepper-Strom',pill(s.stepperPower))+item('A1 AS5600',pill(s.arm1.found))+item('A1 Magnet',pill(s.arm1.magnet))+item('A1 Rohwinkel',s.arm1.rawDeg.toFixed(2)+'°')+item('A1 kalibriert',s.arm1.deg.toFixed(2)+'°')+item('A2 AS5600',pill(s.arm2.found))+item('A2 Magnet',pill(s.arm2.magnet))+item('A2 Rohwinkel',s.arm2.rawDeg.toFixed(2)+'°')+item('A2 kalibriert',s.arm2.deg.toFixed(2)+'°')+item('Endschalter',`MIN ${pill(s.limits.min)} MAX ${pill(s.limits.max)}`);['pw','ph','bx','by'].forEach((id,i)=>{let vals=[s.room.w,s.room.h,s.room.x,s.room.y];if(document.activeElement.id!==id)document.getElementById(id).value=vals[i];});}
function renderPositions(){let h='';for(let i=0;i<8;i++){let p=state.positions[i];h+=`<div class=row><b>P${i+1}</b><span class=small>${p.valid?(p.a1.toFixed(1)+'° / '+p.a2.toFixed(1)+'°'):'leer'}</span><button onclick="api('/api/position?cmd=save&i=${i}')">Speichern</button><button onclick="api('/api/position?cmd=goto&i=${i}')">Anfahren</button></div>`;}document.getElementById('positions').innerHTML=h;let m='';[1,2].forEach(mo=>[10,-10,100,-100,1000,-1000].forEach(st=>m+=`<button onclick="api('/api/motor?m=${mo}&steps=${st}')">M${mo} ${st>0?'+':''}${st}</button>`));document.getElementById('motors').innerHTML=m;}
function cmToPx(x,y){let pad=45,sc=Math.min((map.width-2*pad)/state.room.w,(map.height-2*pad)/state.room.h);return [pad+x*sc,pad+y*sc,sc]}
function pxToCm(px,py){let p=cmToPx(0,0),sc=p[2];return [(px-p[0])/sc,(py-p[1])/sc]}
function draw(){if(!state)return;ctx.clearRect(0,0,map.width,map.height);let [ox,oy,sc]=cmToPx(0,0),w=state.room.w*sc,h=state.room.h*sc;ctx.strokeStyle='#6ca6ff';ctx.lineWidth=2;ctx.strokeRect(ox,oy,w,h);let [bx,by]=cmToPx(state.room.x,state.room.y);ctx.strokeStyle='#324a70';ctx.beginPath();ctx.arc(bx,by,(40)*sc,0,Math.PI*2);ctx.stroke();ctx.fillStyle='#ff5d73aa';state.barriers.forEach(b=>{if(b.e){let p=cmToPx(b.x,b.y);ctx.fillRect(p[0],p[1],b.w*sc,b.h*sc)}});let a1=state.arm1.deg*Math.PI/180,a2=(state.arm1.deg+(state.arm2Virtual))*Math.PI/180;let jx=state.room.x+20*Math.cos(a1),jy=state.room.y+20*Math.sin(a1),tx=jx+20*Math.cos(a2),ty=jy+20*Math.sin(a2);let j=cmToPx(jx,jy),t=cmToPx(tx,ty);ctx.lineWidth=8;ctx.lineCap='round';ctx.strokeStyle='#2fe39b';ctx.beginPath();ctx.moveTo(bx,by);ctx.lineTo(j[0],j[1]);ctx.lineTo(t[0],t[1]);ctx.stroke();ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(bx,by,5,0,7);ctx.arc(j[0],j[1],5,0,7);ctx.arc(t[0],t[1],7,0,7);ctx.fill();info();}
function info(){document.getElementById('hint').textContent=barrierMode?'Barriere: auf Karte ziehen.':'Klick auf Karte = Smart-Zielpunkt. Erreichbarer Radius ist eingezeichnet.'}
map.addEventListener('mousedown',e=>{let r=map.getBoundingClientRect(),x=(e.clientX-r.left)*map.width/r.width,y=(e.clientY-r.top)*map.height/r.height;if(barrierMode)drawStart=[x,y];else{let c=pxToCm(x,y);api(`/api/smart?x=${c[0].toFixed(2)}&y=${c[1].toFixed(2)}`)}});
map.addEventListener('mouseup',e=>{if(!barrierMode||!drawStart)return;let r=map.getBoundingClientRect(),x=(e.clientX-r.left)*map.width/r.width,y=(e.clientY-r.top)*map.height/r.height;let a=pxToCm(drawStart[0],drawStart[1]),b=pxToCm(x,y);drawStart=null;api(`/api/barrier?cmd=add&x=${Math.min(a[0],b[0]).toFixed(2)}&y=${Math.min(a[1],b[1]).toFixed(2)}&w=${Math.abs(a[0]-b[0]).toFixed(2)}&h=${Math.abs(a[1]-b[1]).toFixed(2)}`)});
function saveRoom(){api(`/api/room?w=${pw.value}&h=${ph.value}&x=${bx.value}&y=${by.value}`)}function clearBarriers(){api('/api/barrier?cmd=clear')}setInterval(load,1000);load();
</script></body></html>
)rawliteral";

String statusJson() {
  updateEncoders();
  String j = "{";
  j += "\"wifi\":" + yn(WiFi.status() == WL_CONNECTED) + ",";
  j += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  j += "\"n2\":" + yn(n2Reachable) + ",";
  j += "\"stepperPower\":" + yn(stepperPowerOn) + ",";
  j += "\"arm1\":{";
  j += "\"found\":" + yn(enc1.found) + ",\"magnet\":" + yn(enc1.magnetDetected) + ",\"raw\":" + String(enc1.raw) + ",\"rawDeg\":" + String(enc1.rawDeg, 2) + ",\"deg\":" + String(enc1.calibratedDeg, 2) + "},";
  j += "\"arm2\":{";
  j += "\"found\":" + yn(enc2.found) + ",\"magnet\":" + yn(enc2.magnetDetected) + ",\"raw\":" + String(enc2.raw) + ",\"rawDeg\":" + String(enc2.rawDeg, 2) + ",\"deg\":" + String(enc2.calibratedDeg, 2) + "},";
  j += "\"arm2Virtual\":" + String(arm2ToVirtual(enc2.calibratedDeg), 2) + ",";
  j += "\"limits\":{\"min\":" + yn(limitMinPressed()) + ",\"max\":" + yn(limitMaxPressed()) + "},";
  j += "\"room\":{\"w\":" + String(plateW, 2) + ",\"h\":" + String(plateH, 2) + ",\"x\":" + String(baseX, 2) + ",\"y\":" + String(baseY, 2) + "},";
  j += "\"positions\":[";
  for (uint8_t i = 0; i < POSITION_COUNT; i++) {
    if (i) j += ",";
    j += "{\"valid\":" + yn(positions[i].valid) + ",\"a1\":" + String(positions[i].arm1Deg, 2) + ",\"a2\":" + String(positions[i].arm2Deg, 2) + "}";
  }
  j += "],\"barriers\":[";
  for (uint8_t i = 0; i < BARRIER_COUNT; i++) {
    if (i) j += ",";
    j += "{\"e\":" + yn(barriers[i].enabled) + ",\"x\":" + String(barriers[i].x, 2) + ",\"y\":" + String(barriers[i].y, 2) + ",\"w\":" + String(barriers[i].w, 2) + ",\"h\":" + String(barriers[i].h, 2) + "}";
  }
  j += "],\"message\":\"" + jsonEscape(lastMessage) + "\"}";
  return j;
}

void sendText(const String &s, int code = 200) {
  server.send(code, "text/plain; charset=utf-8", s);
}

void handleCalibrate() {
  updateEncoders();
  String cmd = server.arg("cmd");
  if (cmd == "a1zero" && enc1.found) { arm1RawZero = enc1.raw; prefs.putUShort("a1z", arm1RawZero); lastMessage = "Arm 1: 0 Grad gespeichert"; }
  else if (cmd == "a1270" && enc1.found) { arm1Raw270 = enc1.raw; prefs.putUShort("a1270", arm1Raw270); lastMessage = "Arm 1: 270 Grad gespeichert"; }
  else if (cmd == "a2zero" && enc2.found) { arm2RawZero = enc2.raw; prefs.putUShort("a2z", arm2RawZero); lastMessage = "Arm 2: 0 Grad gespeichert"; }
  else if (cmd == "a2min" && enc2.found) { arm2SoftMinDeg = enc2.calibratedDeg; prefs.putFloat("a2min", arm2SoftMinDeg); lastMessage = "Arm 2: Software-MIN gespeichert"; }
  else if (cmd == "a2max" && enc2.found) { arm2SoftMaxDeg = enc2.calibratedDeg; prefs.putFloat("a2max", arm2SoftMaxDeg); lastMessage = "Arm 2: Software-MAX gespeichert"; }
  else { sendText("Kalibrierung fehlgeschlagen", 400); return; }
  sendText(lastMessage);
}

void handleMotor() {
  uint8_t m = (uint8_t)server.arg("m").toInt();
  long steps = server.arg("steps").toInt();
  if ((m != 1 && m != 2) || steps == 0) { sendText("Ungueltiger Motor-Test", 400); return; }
  bool ok = moveMotorSteps(m, steps);
  sendText(lastMessage, ok ? 200 : 409);
}

void handleReverse() {
  uint8_t m = (uint8_t)server.arg("motor").toInt();
  if (m == 1) { motor1Reverse = !motor1Reverse; prefs.putBool("m1rev", motor1Reverse); lastMessage = "Motor-1-Richtung gespeichert"; }
  else if (m == 2) { motor2Reverse = !motor2Reverse; prefs.putBool("m2rev", motor2Reverse); lastMessage = "Motor-2-Richtung gespeichert"; }
  else { sendText("Ungueltiger Motor", 400); return; }
  sendText(lastMessage);
}

void handlePosition() {
  String cmd = server.arg("cmd");
  int i = server.arg("i").toInt();
  if (i < 0 || i >= POSITION_COUNT) { sendText("Ungueltige Position", 400); return; }
  updateEncoders();
  if (cmd == "save") {
    positions[i].valid = true;
    positions[i].arm1Deg = enc1.calibratedDeg;
    positions[i].arm2Deg = enc2.calibratedDeg;
    savePositions();
    lastMessage = "Position gespeichert";
    sendText(lastMessage);
  } else if (cmd == "goto" && positions[i].valid) {
    bool safe = pathIsSafe(enc1.calibratedDeg, enc2.calibratedDeg, positions[i].arm1Deg, positions[i].arm2Deg);
    bool ok = safe && moveToAngles(positions[i].arm1Deg, positions[i].arm2Deg);
    if (!safe) lastMessage = "Position abgelehnt: Pfad/Barriere unsicher";
    sendText(lastMessage, ok ? 200 : 409);
  } else sendText("Position leer oder Befehl ungueltig", 400);
}

void handleRoom() {
  plateW = clampFloat(server.arg("w").toFloat(), 20.0f, 200.0f);
  plateH = clampFloat(server.arg("h").toFloat(), 20.0f, 200.0f);
  baseX = clampFloat(server.arg("x").toFloat(), 0.0f, plateW);
  baseY = clampFloat(server.arg("y").toFloat(), 0.0f, plateH);
  prefs.putFloat("plateW", plateW);
  prefs.putFloat("plateH", plateH);
  prefs.putFloat("baseX", baseX);
  prefs.putFloat("baseY", baseY);
  lastMessage = "Virtueller Raum gespeichert";
  sendText(lastMessage);
}

void handleBarrier() {
  String cmd = server.arg("cmd");
  if (cmd == "clear") {
    for (uint8_t i = 0; i < BARRIER_COUNT; i++) barriers[i].enabled = false;
    saveBarriers();
    lastMessage = "Barrieren geloescht";
    sendText(lastMessage);
    return;
  }
  if (cmd == "add") {
    for (uint8_t i = 0; i < BARRIER_COUNT; i++) {
      if (!barriers[i].enabled) {
        barriers[i].enabled = true;
        barriers[i].x = clampFloat(server.arg("x").toFloat(), 0.0f, plateW);
        barriers[i].y = clampFloat(server.arg("y").toFloat(), 0.0f, plateH);
        barriers[i].w = clampFloat(server.arg("w").toFloat(), 0.5f, plateW);
        barriers[i].h = clampFloat(server.arg("h").toFloat(), 0.5f, plateH);
        saveBarriers();
        lastMessage = "Barriere gespeichert";
        sendText(lastMessage);
        return;
      }
    }
    sendText("Maximale Anzahl Barrieren erreicht", 409);
    return;
  }
  sendText("Ungueltiger Barrieren-Befehl", 400);
}

void handleSmart() {
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  bool ok = smartMoveTo(x, y);
  sendText(lastMessage, ok ? 200 : 409);
}

void setupWeb() {
  server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.on("/api/status", []() { server.send(200, "application/json; charset=utf-8", statusJson()); });
  server.on("/api/calibrate", handleCalibrate);
  server.on("/api/motor", handleMotor);
  server.on("/api/reverse", handleReverse);
  server.on("/api/position", handlePosition);
  server.on("/api/room", handleRoom);
  server.on("/api/barrier", handleBarrier);
  server.on("/api/smart", handleSmart);
  server.on("/api/emergency", []() { emergencyStop(); sendText(lastMessage); });
  server.onNotFound([]() { sendText("Nicht gefunden", 404); });
  server.begin();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    delay(250);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_M1_STEP, OUTPUT);
  pinMode(PIN_M1_DIR, OUTPUT);
  pinMode(PIN_M2_STEP, OUTPUT);
  pinMode(PIN_M2_DIR, OUTPUT);
  pinMode(PIN_A1_MIN, INPUT_PULLUP);
  pinMode(PIN_A1_MAX, INPUT_PULLUP);
  digitalWrite(PIN_M1_STEP, LOW);
  digitalWrite(PIN_M2_STEP, LOW);

  Wire.begin(ARM1_SDA, ARM1_SCL);
  Wire.setClock(400000);
  WireArm2.begin(ARM2_SDA, ARM2_SCL);
  WireArm2.setClock(400000);

  loadSettings();
  connectWiFi();
  updateEncoders();
  httpGetN2("/api/stepperpower?state=0");
  stepperPowerOn = false;
  setupWeb();
  lastMessage = WiFi.status() == WL_CONNECTED ? String("Bereit: ") + WiFi.localIP().toString() : "Bereit, aber WLAN nicht verbunden";
}

void loop() {
  server.handleClient();
  static uint32_t lastSensor = 0;
  static uint32_t lastN2 = 0;
  if (millis() - lastSensor > 250) {
    lastSensor = millis();
    updateEncoders();
  }
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifiTry = 0;
    if (millis() - lastWifiTry > 5000) {
      lastWifiTry = millis();
      WiFi.reconnect();
    }
  }
  if (millis() - lastN2 > 10000) {
    lastN2 = millis();
    httpGetN2("/api/stepperpower?state=0");
    stepperPowerOn = false;
  }
}
