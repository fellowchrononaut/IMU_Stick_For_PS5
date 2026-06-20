/**
 * IMU Stick V1 firmware
 * - BNO055 over I2C (address 0x28, NDOF mode)
 * - Two ESP32 DAC channels driving a PS5 Access Controller stick via op-amp
 * - Configurable axis mapping (source, invert, max angle, deadzone) per channel
 * - Neutral pose calibration
 * - Multiple named profiles persisted in NVS
 * - BNO055 sensor calibration offsets persisted in NVS (skip re-cal on boot)
 *
 * Serial protocol (line-based, 115200 baud):
 *   Host -> ESP32:
 *     STATUS                              dump current config (CFG line)
 *     SET X.src=heading Y.inv=1 ...       update any subset of X/Y fields in RAM
 *     NEUTRAL                             capture current pose as baseline (RAM)
 *     SAVE                                persist current RAM config to active profile
 *     LOAD                                reload active profile from NVS into RAM
 *     RESET                               restore defaults in RAM (Save to persist)
 *     LIST_PROFILES                       reply: PROFILES <comma list>
 *     SELECT_PROFILE <name>               load profile into RAM, set active
 *     SAVE_PROFILE <name>                 save RAM config as <name>, mark active
 *     DELETE_PROFILE <name>               remove named profile
 *     SAVE_BNO_CAL                        snapshot BNO055 cal offsets to NVS
 *     CLEAR_BNO_CAL                       remove saved BNO055 cal
 *   ESP32 -> host:
 *     READY imu_stick_v1                  on boot
 *     CFG ...                             reply to STATUS; includes profile=, bno_cal_saved=
 *     PROFILES <name1,name2,...>          reply to LIST_PROFILES (empty -> "PROFILES")
 *     OK ... / ERR ...                    command replies
 *     D h=.. p=.. r=.. dx=.. dy=.. cal=s,g,a,m baseline_set=0|1
 *                                         streamed at ~10 Hz
 *
 * NVS keys (namespace "imuv1"):
 *   active        active profile name (string, default "default")
 *   prof_list     comma-separated profile names
 *   p_<name>      Config blob (per profile)
 *   bno_cal       22-byte BNO055 calibration offset blob
 */

#include "BNO055_support.h"
#include <Wire.h>
#include <Preferences.h>

#define DAC_X 25                    // Ring 1 on TRRS (stick X)
#define DAC_Y 26                    // Tip on TRRS (stick Y)
#define BNO055_SAMPLERATE_DELAY_MS 100
#define CONFIG_MAGIC 0xC0DE
#define BNO055_I2C_ADDRESS 0x28
#define BNO055_CAL_REG_START 0x55
#define BNO055_CAL_BLOB_SIZE 22
#define PROFILE_NAME_MAX 12

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
char activeProfile[PROFILE_NAME_MAX + 1] = "default";
bool bnoCalSaved = false;

struct bno055_t myBNO;
struct bno055_euler myEulerData;
unsigned char sysCal = 0, gyroCal = 0, accelCal = 0, magCal = 0;

unsigned long lastSample = 0;
String inputLine;
Preferences prefs;

// ---------------- defaults ----------------
void loadDefaults() {
  cfg.x = { SRC_HEADING, 0, 25.0f, 0.0f };
  cfg.y = { SRC_ROLL,    0, 25.0f, 0.0f };
  cfg.baseline_heading = 0;
  cfg.baseline_pitch = 0;
  cfg.baseline_roll = 0;
  cfg.baseline_set = 0;
  cfg.magic = CONFIG_MAGIC;
}

// ---------------- profile name validation ----------------
bool isValidProfileName(const String& name) {
  if (name.length() == 0 || name.length() > PROFILE_NAME_MAX) return false;
  for (size_t i = 0; i < name.length(); i++) {
    char c = name[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

String profileKey(const char* name) {
  String k = "p_";
  k += name;
  return k;
}

// ---------------- profile list management ----------------
String readProfileList() {
  prefs.begin("imuv1", true);
  String list = prefs.getString("prof_list", "");
  prefs.end();
  return list;
}

void writeProfileList(const String& list) {
  prefs.begin("imuv1", false);
  prefs.putString("prof_list", list);
  prefs.end();
}

bool profileExistsInList(const String& list, const char* name) {
  String needle = name;
  // search for comma-delimited token
  int from = 0;
  while (from <= (int)list.length()) {
    int comma = list.indexOf(',', from);
    String token = (comma < 0) ? list.substring(from) : list.substring(from, comma);
    if (token == needle) return true;
    if (comma < 0) break;
    from = comma + 1;
  }
  return false;
}

String addToList(const String& list, const char* name) {
  if (profileExistsInList(list, name)) return list;
  if (list.length() == 0) return String(name);
  return list + "," + name;
}

String removeFromList(const String& list, const char* name) {
  String out = "";
  int from = 0;
  while (from <= (int)list.length()) {
    int comma = list.indexOf(',', from);
    String token = (comma < 0) ? list.substring(from) : list.substring(from, comma);
    if (token != name && token.length() > 0) {
      if (out.length() > 0) out += ",";
      out += token;
    }
    if (comma < 0) break;
    from = comma + 1;
  }
  return out;
}

// ---------------- profile persistence ----------------
bool loadProfileByName(const char* name) {
  Config tmp;
  prefs.begin("imuv1", true);
  size_t n = prefs.getBytes(profileKey(name).c_str(), &tmp, sizeof(tmp));
  prefs.end();
  if (n != sizeof(tmp) || tmp.magic != CONFIG_MAGIC) return false;
  cfg = tmp;
  return true;
}

void saveProfileByName(const char* name) {
  cfg.magic = CONFIG_MAGIC;
  prefs.begin("imuv1", false);
  prefs.putBytes(profileKey(name).c_str(), &cfg, sizeof(cfg));
  prefs.end();
  String list = readProfileList();
  String updated = addToList(list, name);
  if (updated != list) writeProfileList(updated);
}

void deleteProfileByName(const char* name) {
  prefs.begin("imuv1", false);
  prefs.remove(profileKey(name).c_str());
  prefs.end();
  String list = readProfileList();
  String updated = removeFromList(list, name);
  if (updated != list) writeProfileList(updated);
}

// ---------------- active profile pointer ----------------
void readActiveName(char* out) {
  prefs.begin("imuv1", true);
  String s = prefs.getString("active", "default");
  prefs.end();
  s.toCharArray(out, PROFILE_NAME_MAX + 1);
}

void writeActiveName(const char* name) {
  prefs.begin("imuv1", false);
  prefs.putString("active", name);
  prefs.end();
}

// ---------------- BNO055 raw I2C helpers for cal blob ----------------
bool readBNOCalRegisters(uint8_t* buf) {
  bno055_set_operation_mode(OPERATION_MODE_CONFIG);
  delay(25);
  Wire.beginTransmission(BNO055_I2C_ADDRESS);
  Wire.write(BNO055_CAL_REG_START);
  if (Wire.endTransmission(false) != 0) {
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(25);
    return false;
  }
  size_t got = Wire.requestFrom((uint8_t)BNO055_I2C_ADDRESS, (size_t)BNO055_CAL_BLOB_SIZE);
  for (size_t i = 0; i < BNO055_CAL_BLOB_SIZE; i++) {
    buf[i] = (i < got && Wire.available()) ? Wire.read() : 0;
  }
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(25);
  return got == BNO055_CAL_BLOB_SIZE;
}

bool writeBNOCalRegisters(const uint8_t* buf) {
  bno055_set_operation_mode(OPERATION_MODE_CONFIG);
  delay(25);
  Wire.beginTransmission(BNO055_I2C_ADDRESS);
  Wire.write(BNO055_CAL_REG_START);
  for (size_t i = 0; i < BNO055_CAL_BLOB_SIZE; i++) {
    Wire.write(buf[i]);
  }
  bool ok = (Wire.endTransmission() == 0);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(25);
  return ok;
}

bool loadBNOCalFromNVS(uint8_t* buf) {
  prefs.begin("imuv1", true);
  size_t n = prefs.getBytes("bno_cal", buf, BNO055_CAL_BLOB_SIZE);
  prefs.end();
  return n == BNO055_CAL_BLOB_SIZE;
}

void saveBNOCalToNVS(const uint8_t* buf) {
  prefs.begin("imuv1", false);
  prefs.putBytes("bno_cal", buf, BNO055_CAL_BLOB_SIZE);
  prefs.end();
}

void clearBNOCalFromNVS() {
  prefs.begin("imuv1", false);
  prefs.remove("bno_cal");
  prefs.end();
}

// ---------------- mapping helpers ----------------
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

  if (ax.src == SRC_HEADING) {
    if (delta > 180.0f)  delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
  }

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

// ---------------- replies ----------------
void printStatus() {
  Serial.print(F("CFG"));
  Serial.print(F(" profile=")); Serial.print(activeProfile);
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
  Serial.print(F(" baseline_set=")); Serial.print(cfg.baseline_set);
  Serial.print(F(" bno_cal_saved=")); Serial.println(bnoCalSaved ? 1 : 0);
}

void printProfiles() {
  Serial.print(F("PROFILES "));
  Serial.println(readProfileList());
}

// ---------------- SET parser ----------------
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

// ---------------- command dispatch ----------------
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
    saveProfileByName(activeProfile);
    Serial.print(F("OK SAVED profile=")); Serial.println(activeProfile);
  } else if (cmd == "LOAD") {
    bool ok = loadProfileByName(activeProfile);
    Serial.println(ok ? F("OK LOADED") : F("OK DEFAULTS (no saved profile)"));
  } else if (cmd == "RESET") {
    loadDefaults();
    Serial.println(F("OK RESET (not saved)"));
  } else if (cmd == "LIST_PROFILES") {
    printProfiles();
  } else if (cmd.startsWith("SELECT_PROFILE ")) {
    String name = cmd.substring(15); name.trim();
    if (!isValidProfileName(name)) {
      Serial.println(F("ERR invalid profile name"));
      return;
    }
    if (!loadProfileByName(name.c_str())) {
      Serial.print(F("ERR profile not found: ")); Serial.println(name);
      return;
    }
    name.toCharArray(activeProfile, PROFILE_NAME_MAX + 1);
    writeActiveName(activeProfile);
    Serial.print(F("OK SELECTED ")); Serial.println(activeProfile);
  } else if (cmd.startsWith("SAVE_PROFILE ")) {
    String name = cmd.substring(13); name.trim();
    if (!isValidProfileName(name)) {
      Serial.println(F("ERR invalid profile name"));
      return;
    }
    saveProfileByName(name.c_str());
    name.toCharArray(activeProfile, PROFILE_NAME_MAX + 1);
    writeActiveName(activeProfile);
    Serial.print(F("OK SAVED_PROFILE ")); Serial.println(activeProfile);
  } else if (cmd.startsWith("DELETE_PROFILE ")) {
    String name = cmd.substring(15); name.trim();
    if (!isValidProfileName(name)) {
      Serial.println(F("ERR invalid profile name"));
      return;
    }
    if (name == activeProfile) {
      Serial.println(F("ERR cannot delete active profile"));
      return;
    }
    deleteProfileByName(name.c_str());
    Serial.print(F("OK DELETED ")); Serial.println(name);
  } else if (cmd == "SAVE_BNO_CAL") {
    uint8_t buf[BNO055_CAL_BLOB_SIZE];
    if (!readBNOCalRegisters(buf)) {
      Serial.println(F("ERR could not read BNO cal"));
      return;
    }
    saveBNOCalToNVS(buf);
    bnoCalSaved = true;
    Serial.println(F("OK SAVED_BNO_CAL"));
  } else if (cmd == "CLEAR_BNO_CAL") {
    clearBNOCalFromNVS();
    bnoCalSaved = false;
    Serial.println(F("OK CLEARED_BNO_CAL"));
  } else {
    Serial.print(F("ERR unknown: "));
    Serial.println(cmd);
  }
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  delay(800);                                       // BNO055 power-on settling
  BNO_Init(&myBNO);

  // If a saved BNO cal exists, write it back to the chip before going into NDOF.
  uint8_t calBuf[BNO055_CAL_BLOB_SIZE];
  if (loadBNOCalFromNVS(calBuf)) {
    writeBNOCalRegisters(calBuf);                   // also leaves us in NDOF
    bnoCalSaved = true;
  } else {
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(30);
    bnoCalSaved = false;
  }

  // Active profile.
  readActiveName(activeProfile);
  if (!loadProfileByName(activeProfile)) {
    loadDefaults();                                 // first-ever boot or wiped NVS
    saveProfileByName(activeProfile);
  }

  dacWrite(DAC_X, 128);
  dacWrite(DAC_Y, 128);

  Serial.println(F("READY imu_stick_v1"));
}

// ---------------- loop ----------------
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
      if (inputLine.length() > 200) inputLine = "";
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