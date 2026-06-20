/**
 * IMU Stick V1 firmware
 * - BNO055 over I2C
 * - Two ESP32 DAC channels driving a PS5 Access Controller stick via op-amp
 * - Configurable axis mapping (source, invert, max angle, deadzone) per channel
 * - Neutral pose calibration
 * - Persistent config in NVS so the device runs standalone once configured
 *
 * Serial protocol (line-based, 115200 baud):
 *   Host -> ESP32:
 *     STATUS                              request current config
 *     SET X.src=heading Y.inv=1 X.max=20  any subset of X/Y fields
 *     NEUTRAL                             capture current pose as baseline (RAM)
 *     SAVE                                persist RAM config to NVS
 *     LOAD                                reload NVS into RAM
 *     RESET                               restore defaults in RAM (Save to persist)
 *   ESP32 -> host:
 *     READY imu_stick_v1                  printed once on boot
 *     CFG ...                             reply to STATUS
 *     OK ... / ERR ...                    reply to commands
 *     D h=.. p=.. r=.. dx=.. dy=.. cal=s,g,a,m baseline_set=0|1
 *                                         streamed at ~10 Hz
 */

#include "BNO055_support.h"
#include <Wire.h>
#include <Preferences.h>

#define DAC_X 25                    // Ring 1 on TRRS (stick X)
#define DAC_Y 26                    // Tip on TRRS (stick Y)
#define BNO055_SAMPLERATE_DELAY_MS 100
#define CONFIG_MAGIC 0xC0DE

enum AxisSource : uint8_t { SRC_HEADING = 0, SRC_PITCH = 1, SRC_ROLL = 2 };

struct AxisConfig {
  uint8_t src;
  uint8_t invert;
  float max_angle;
  float dead_zone;
};

struct Config {
  AxisConfig x;
  AxisConfig y;
  float baseline_heading;
  float baseline_pitch;
  float baseline_roll;
  uint8_t baseline_set;
  uint16_t magic;
};

Config cfg;

struct bno055_t myBNO;
struct bno055_euler myEulerData;
unsigned char sysCal = 0, gyroCal = 0, accelCal = 0, magCal = 0;

unsigned long lastSample = 0;
String inputLine;
Preferences prefs;

void loadDefaults() {
  cfg.x = { SRC_HEADING, 0, 25.0f, 0.0f };
  cfg.y = { SRC_ROLL,    0, 25.0f, 0.0f };
  cfg.baseline_heading = 0;
  cfg.baseline_pitch = 0;
  cfg.baseline_roll = 0;
  cfg.baseline_set = 0;
  cfg.magic = CONFIG_MAGIC;
}

bool loadFromNVS() {
  Config tmp;
  prefs.begin("imuv1", true);
  size_t n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  prefs.end();
  if (n != sizeof(tmp) || tmp.magic != CONFIG_MAGIC) {
    loadDefaults();
    return false;
  }
  cfg = tmp;
  return true;
}

void saveToNVS() {
  cfg.magic = CONFIG_MAGIC;
  prefs.begin("imuv1", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}

const char* srcName(uint8_t s) {
  if (s == SRC_HEADING) return "heading";
  if (s == SRC_PITCH)   return "pitch";
  if (s == SRC_ROLL)    return "roll";
  return "?";
}

int srcCode(const String& name) {
  if (name == "heading") return SRC_HEADING;
  if (name == "pitch")   return SRC_PITCH;
  if (name == "roll")    return SRC_ROLL;
  return -1;
}

float pickAxisValue(uint8_t src, float h, float p, float r) {
  if (src == SRC_HEADING) return h;
  if (src == SRC_PITCH)   return p;
  return r;
}

float pickBaseline(uint8_t src) {
  if (src == SRC_HEADING) return cfg.baseline_heading;
  if (src == SRC_PITCH)   return cfg.baseline_pitch;
  return cfg.baseline_roll;
}

int computeDAC(const AxisConfig& ax, float h, float p, float r) {
  float cur = pickAxisValue(ax.src, h, p, r);
  float base = pickBaseline(ax.src);
  float delta = cur - base;

  // heading wrap-around (-180..180)
  if (ax.src == SRC_HEADING) {
    if (delta > 180.0f)  delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
  }

  // dead zone — collapse small motions to zero, shift larger motions in toward zero
  if (fabsf(delta) < ax.dead_zone) {
    delta = 0.0f;
  } else if (delta > 0.0f) {
    delta -= ax.dead_zone;
  } else {
    delta += ax.dead_zone;
  }

  if (ax.invert) delta = -delta;

  float maxa = ax.max_angle;
  if (maxa < 1.0f) maxa = 1.0f;
  if (delta >  maxa) delta =  maxa;
  if (delta < -maxa) delta = -maxa;

  float normalized = delta / maxa;
  int dac = (int)((normalized + 1.0f) * 127.5f);
  if (dac < 0)   dac = 0;
  if (dac > 255) dac = 255;
  return dac;
}

void printStatus() {
  Serial.print(F("CFG"));
  Serial.print(F(" X.src=")); Serial.print(srcName(cfg.x.src));
  Serial.print(F(" X.inv=")); Serial.print(cfg.x.invert);
  Serial.print(F(" X.max=")); Serial.print(cfg.x.max_angle, 2);
  Serial.print(F(" X.dz="));  Serial.print(cfg.x.dead_zone, 2);
  Serial.print(F(" Y.src=")); Serial.print(srcName(cfg.y.src));
  Serial.print(F(" Y.inv=")); Serial.print(cfg.y.invert);
  Serial.print(F(" Y.max=")); Serial.print(cfg.y.max_angle, 2);
  Serial.print(F(" Y.dz="));  Serial.print(cfg.y.dead_zone, 2);
  Serial.print(F(" baseline="));
  Serial.print(cfg.baseline_heading, 2); Serial.print(',');
  Serial.print(cfg.baseline_pitch, 2);   Serial.print(',');
  Serial.print(cfg.baseline_roll, 2);
  Serial.print(F(" baseline_set=")); Serial.println(cfg.baseline_set);
}

void applyField(AxisConfig& ax, const String& field, const String& val) {
  if (field == "src") {
    int c = srcCode(val);
    if (c >= 0) ax.src = c;
  } else if (field == "inv") {
    ax.invert = (val == "1") ? 1 : 0;
  } else if (field == "max") {
    float v = val.toFloat();
    if (v >= 1.0f && v <= 180.0f) ax.max_angle = v;
  } else if (field == "dz") {
    float v = val.toFloat();
    if (v >= 0.0f && v <= 45.0f) ax.dead_zone = v;
  }
}

void handleSet(const String& args) {
  int start = 0;
  while (start < (int)args.length()) {
    int space = args.indexOf(' ', start);
    if (space < 0) space = args.length();
    String pair = args.substring(start, space);
    int eq = pair.indexOf('=');
    if (eq > 0) {
      String key = pair.substring(0, eq);
      String val = pair.substring(eq + 1);
      if (key.startsWith("X.")) {
        applyField(cfg.x, key.substring(2), val);
      } else if (key.startsWith("Y.")) {
        applyField(cfg.y, key.substring(2), val);
      }
    }
    start = space + 1;
  }
  Serial.println(F("OK SET"));
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "STATUS") {
    printStatus();
  } else if (cmd.startsWith("SET ")) {
    handleSet(cmd.substring(4));
  } else if (cmd == "NEUTRAL") {
    bno055_read_euler_hrp(&myEulerData);
    cfg.baseline_heading = (float)myEulerData.h / 16.0f;
    cfg.baseline_pitch   = (float)myEulerData.p / 16.0f;
    cfg.baseline_roll    = (float)myEulerData.r / 16.0f;
    cfg.baseline_set = 1;
    Serial.print(F("OK NEUTRAL baseline="));
    Serial.print(cfg.baseline_heading, 2); Serial.print(',');
    Serial.print(cfg.baseline_pitch, 2);   Serial.print(',');
    Serial.println(cfg.baseline_roll, 2);
  } else if (cmd == "SAVE") {
    saveToNVS();
    Serial.println(F("OK SAVED"));
  } else if (cmd == "LOAD") {
    bool ok = loadFromNVS();
    Serial.println(ok ? F("OK LOADED") : F("OK DEFAULTS (no saved config)"));
  } else if (cmd == "RESET") {
    loadDefaults();
    Serial.println(F("OK RESET (not saved)"));
  } else {
    Serial.print(F("ERR unknown: "));
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  delay(800);                                      // BNO055 power-on settling
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(30);                                       // NDOF mode settle

  loadFromNVS();                                   // populates cfg with saved or defaults

  dacWrite(DAC_X, 128);
  dacWrite(DAC_Y, 128);

  Serial.println(F("READY imu_stick_v1"));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
      if (inputLine.length() > 200) inputLine = "";  // guard against runaway
    }
  }

  if ((millis() - lastSample) < BNO055_SAMPLERATE_DELAY_MS) return;
  lastSample = millis();

  bno055_read_euler_hrp(&myEulerData);
  float h = (float)myEulerData.h / 16.0f;
  float p = (float)myEulerData.p / 16.0f;
  float r = (float)myEulerData.r / 16.0f;

  int dx = 128, dy = 128;
  if (cfg.baseline_set) {
    dx = computeDAC(cfg.x, h, p, r);
    dy = computeDAC(cfg.y, h, p, r);
    dacWrite(DAC_X, dx);
    dacWrite(DAC_Y, dy);
  }

  bno055_get_syscalib_status(&sysCal);
  bno055_get_gyrocalib_status(&gyroCal);
  bno055_get_accelcalib_status(&accelCal);
  bno055_get_magcalib_status(&magCal);

  Serial.print(F("D h="));  Serial.print(h, 2);
  Serial.print(F(" p="));   Serial.print(p, 2);
  Serial.print(F(" r="));   Serial.print(r, 2);
  Serial.print(F(" dx="));  Serial.print(dx);
  Serial.print(F(" dy="));  Serial.print(dy);
  Serial.print(F(" cal=")); Serial.print(sysCal);
  Serial.print(',');        Serial.print(gyroCal);
  Serial.print(',');        Serial.print(accelCal);
  Serial.print(',');        Serial.print(magCal);
  Serial.print(F(" baseline_set=")); Serial.println(cfg.baseline_set);
}