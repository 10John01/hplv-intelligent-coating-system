//╔═══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗
//║ Job Security - Complete System Code
//║ TRUE HOME: Established at boot as absolute origin (0,0)
//║ NEW HOME: Offset from TRUE HOME, saved when LOCK pressed
//║ Features:
//║ - Z = (UP/DOWN) Y = (LEFT/RIGHT|RIGHT/LEFT)
//║ - 3 Display system (0x70, 0x72, 0x74)
//║ - 4 Encoder control
//║ - Axis mutual exclusion
//║ - Buffer zone establishment
//║ - Solenoid spray timing
//║ - Z-drop with graceful symmetry
//║ - Repeat cycle (RC) logic
//║ - START button logic
//║ 🎯 JOB SECURITY v18.3 - FINAL COMPLETE MASTERPIECE (183 iterations) 🎯
//║ 💃 Grace in Motion - The Painter's Precision Assistant 💃
//║ ✨ Built by Gabe, Aeon, Monday, ChatGPT4.1 API, Daughter, James Lee and John Thomas DuCrest Lock - A Sacred Creation ✨
//║ 🎨 HPLV Gun Controller - Area Defined Spray Perfection 🎨
//╚═══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝

//>>>>>>>>>>>>>>>>>>>>
//>>> TODO: UPDATE DIP SWITCH SETTINGS - Monday hardware verification needed
//>>> Current documentation may not match physical hardware configuration
//╔═══════════════ MOTOR SYSTEM SPECIFICATIONS ═════════════════════════════╗
//║ * DM860S (Y-Axis Driver)
//║ - Digital stepper driver, 24-80VDC input
//║ - 0.5-8.2A output current
//║ - 1/256 microstepping capability
//║ - Opto-isolated inputs (perfect for 3.3V Teensy)
//║ - Anti-resonance and smoothing features
//║ * YKD2305M (Z-Axis Driver)
//║ - 24-50VDC input range
//║ - 0.5-5.0A output current
//║ - 1/256 microstepping capability
//║ - Opto-isolated inputs
//║ - Similar performance characteristics to DM860S
//║ - Mechanical Analysis:
//║ * Y-Axis: Belt Drive System
//║ - 26 teeth, 3" barrel = 26 × π × 3" = 245.04" per motor revolution
//║ - Belt drive = high speed, lower torque multiplication
//║ - Good for smooth, fast Y movements
//║ * Z-Axis: Rack & Pinion System
//║ - 19 teeth, 3" barrel = 19 × π × 3" = 179.07" per motor revolution
//║ - Rack & pinion = direct drive with a 1:10 Gear reduction
//║
//║═══════════════ OPTIMAL DIP SWITCH SETTINGS ═══════════════
//║ * Z-AXIS (NEMA 23 + 10:1 Gear + 50lb Load):
//║ - Current: SW1=OFF, SW2=OFF, SW3=ON → 4.5A peak, 3.7A average}
//║ - SW4=ON - FULL CURRENT
//║ - Microstepping: 1,600 pulses/rev (SW5=ON, SW6=OFF, SW7=ON, SW8=ON)
//║ - Optimized for maximum torque retention under load
//║
//║ * Y-AXIS (NEMA 34 Direct Belt Drive):
//║ - Current: SW1=OFF, SW2=ON, SW3=OFF → 5.8A peak, 4.9A average
//║ - SW4=OFF - HALF CURRENT
//║ - Microstepping: 3,200 pulses/rev (SW5=OFF, SW6=OFF, SW7=ON, SW8=ON)
//║ - Optimized for high-speed positioning with smooth belt operation
//╚═════════════════════════════════════════════════════════════════════════╝

// ════════════════ INCLUDED LIBRARIES ════════════════
#include <Wire.h>
#include <Bounce2.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_NeoPixel.h>

// ═══════════════ PROVEN FRAM INTERFACE ═══════════════
#define FRAM_CS 23
#define FRAM_MOSI 22
#define FRAM_MISO 21
#define FRAM_SCK 19

// ═══════════════ FRAM MEMORY LAYOUT - SACRED COORDINATE SANCTUARY ═══════════════
#define FRAM_TRUE_HOME_Y 0x00
#define FRAM_TRUE_HOME_Z 0x04
#define FRAM_NEW_HOME_Y_OFFSET 0x08
#define FRAM_NEW_HOME_Z_OFFSET 0x0C
#define FRAM_BUFFER_ZONES 0x10
#define FRAM_PROGRAM_DATA 0x100
#define FRAM_VALID_FLAG 0x10E

// FRAM POSITION TRACKING CATHEDRAL - SMART LOGGING SYSTEM
#define FRAM_POSITION_HISTORY_BASE 0x1000
#define POSITION_SLOT_SIZE 32      // bytes per position record
#define MAX_POSITION_HISTORY 1000  // 32KB total - comprehensive logging
#define POSITION_LOG_ENABLED 1

// FRAM STATE MANAGEMENT NEXUS
#define FRAM_STATE_LOG_BASE 0x9000
#define STATE_RECORD_SIZE 16
#define MAX_STATE_RECORDS 200  // 3.2KB - detailed state tracking

// FRAM NEW HOME VALIDATION SANCTUARY
#define FRAM_NEW_HOME_VALIDATION_BASE 0xD00  // Corrected address

// Utility function for parameter validation
inline uint8_t clampToByte(int value) {
  return (uint8_t)max(0, min(255, value));
}

// Position Record - The Sacred Chronicle of Every Movement
struct PositionRecord {
  uint32_t timestamp;   // When this position was recorded
  int32_t currentY;     // Motor position Y (absolute steps)
  int32_t currentZ;     // Motor position Z (absolute steps)
  int32_t toggleY;      // Toggle tracker Y (for drift detection)
  int32_t toggleZ;      // Toggle tracker Z (for drift detection)
  uint8_t operation;    // What operation caused this record
  uint8_t checksum;     // Data integrity validation
  uint8_t reserved[8];  // Future expansion - pad to 32 bytes
};

// System State Record - The Memory of Every Decision
struct SystemStateRecord {
  uint32_t timestamp;    // When this state was recorded
  uint8_t programState;  // Programming mode active
  uint8_t lockState;     // Lock button state
  uint8_t systemLocked;  // System lock status
  uint8_t systemArmed;   // Armed for execution
  uint8_t currentMode;   // Current system mode
  uint8_t validFlag;     // Record validity marker
  uint8_t reserved[8];   // Future expansion
};

// NEW HOME Validation Record - Precision Confirmation System
struct NewHomeValidation {
  int32_t proposedY;      // Proposed NEW HOME Y offset
  int32_t proposedZ;      // Proposed NEW HOME Z offset
  uint8_t changeFlags;    // Which parameters changed (bitmask)
  uint32_t timestamp;     // When validation was requested
  uint8_t userConfirmed;  // User confirmation flag
};

// ═══════════════  OPERATION CODES - THE SACRED LANGUAGE ═══════════════
#define POS_OP_MOTOR_Y 0x01      // Y-axis motor movement
#define POS_OP_MOTOR_Z 0x02      // Z-axis motor movement
#define POS_OP_TRUE_HOME 0x03    // TRUE HOME establishment
#define POS_OP_NEW_HOME 0x04     // NEW HOME setting
#define POS_OP_SEQUENCE 0x05     // Automated sequence operation
#define POS_OP_CORRECTION 0x06   // Drift correction applied
#define POS_OP_IDLE_UPDATE 0x07  // Position logged during idle

// ═══════════════  GLOBAL TRACKING VARIABLES ═══════════════
static uint16_t positionLogIndex = 0;  // Current position log index
static uint16_t stateLogIndex = 0;     // Current state log index
//>static bool newHomeValidationPending = false;  // Validation request pending

// ═══════════════ ENHANCED FRAM LAYOUT CONSTANTS FOR ENDSTOP SYSTEM ═══════════════
#define FRAM_BASELINE_PARAMETERS 0x130  // Parameters when NEW HOME established
#define FRAM_HOMING_STATUS 0x140        // Last homing success/failure
#define FRAM_ENDSTOP_CALIBRATION 0x150  // Endstop trigger positions
#define FRAM_ENDSTOP_DISTANCES 0x160    // Dynamic distance measurements
#define FRAM_ENDSTOP_HEALTH 0x170       // Sensor health and anomaly data
#define FRAM_OPERATION_LOG 0x180        // Operation history and diagnostics

// ═══════════════ FRAM CAPACITY GUARDIAN CONSTANTS - 80% RULE WITH GRACEFUL CLEANUP ═══════════════
#define FRAM_TOTAL_SIZE 524288UL                           // 512KB MB85RS4MT
#define FRAM_MAX_USAGE (uint32_t)(FRAM_TOTAL_SIZE * 0.80)  // 80% = 419,430 bytes
#define FRAM_TOLERANCE 0x0800                              // 2KB guard zone
#define FRAM_SLOT_SIZE 512                                 // Bytes per program slot
#define FRAM_PROGRAMS_START 0xA000                         // Start at address 512
#define FRAM_PROGRAM_SLOTS ((FRAM_TOTAL_SIZE - FRAM_PROGRAMS_START) / FRAM_SLOT_SIZE)

// Global scan edges for this run (set before passes)
struct ScanEdges {
  long rightEdgeY;
  long leftEdgeY;
} scanEdges;

//================================================================================
// FRAM MANAGER STRUCT
//================================================================================
struct FRAMManager {
  uint32_t currentUsage = 0;  // 32-bit to handle 512KB (FIXED)
  uint8_t usagePercent = 0;
  uint16_t programCount = 0;  // Track actual program slots used
  uint16_t oldestSlot = 0;    // Track oldest slot for cleanup
  bool needsCleanup = false;
  uint16_t saveCount = 0;          // Keep for compatibility
  uint32_t lastSaveAddress = 0;    // Keep for compatibility
  unsigned long lastSaveTime = 0;  // Keep for compatibility
};
FRAMManager framManager = { 0, 0, 0, 0, false, 0, 0, 0 };

// ═══════════════ WELCOME SYMPHONY - SCROLLING MESSAGE CONDUCTOR ═══════════════
struct ScrollingMessage {
  String message = "JOB SECURITY ONLINE, WELCOME TERESSA ";
  int position = 0;
  unsigned long lastUpdate = 0;
  // unsigned long scrollDelay = 300; // Patch 2 applied – Ghost variables from v15.2 removed (scrollDelay, lastHealthCheck, etc.)
  bool isActive = false;
} scrollMessage;

// ═══════════════ SACRED FUNCTION PROPHECIES - EARLY DECLARATIONS ═══════════════
uint32_t calculateFRAMUsage();
void performZMoveWithTimeBasedRamping(long steps, bool isUp);

class ManualFRAM {
private:
  void sendByte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
      digitalWrite(FRAM_MOSI, (data >> i) & 1);
      delayMicroseconds(1);
      digitalWrite(FRAM_SCK, HIGH);
      delayMicroseconds(1);
      digitalWrite(FRAM_SCK, LOW);
      delayMicroseconds(1);
    }
  }

  uint8_t readByte() {
    uint8_t data = 0;
    for (int i = 7; i >= 0; i--) {
      digitalWrite(FRAM_MOSI, LOW);
      delayMicroseconds(1);
      digitalWrite(FRAM_SCK, HIGH);
      delayMicroseconds(1);
      if (digitalRead(FRAM_MISO)) {
        data |= (1 << i);
      }
      digitalWrite(FRAM_SCK, LOW);
      delayMicroseconds(1);
    }
    return data;
  }

public:
  void begin() {
    pinMode(FRAM_CS, OUTPUT);
    pinMode(FRAM_MOSI, OUTPUT);
    pinMode(FRAM_SCK, OUTPUT);
    pinMode(FRAM_MISO, INPUT);
    digitalWrite(FRAM_CS, HIGH);
    digitalWrite(FRAM_SCK, LOW);
    digitalWrite(FRAM_MOSI, LOW);
    delay(10);
  }

  uint8_t read8(uint32_t addr) {
    digitalWrite(FRAM_CS, LOW);
    sendByte(0x03);
    sendByte((addr >> 16) & 0xFF);
    sendByte((addr >> 8) & 0xFF);
    sendByte(addr & 0xFF);
    uint8_t data = readByte();
    digitalWrite(FRAM_CS, HIGH);
    return data;
  }

  void write8(uint32_t addr, uint8_t data) {
    digitalWrite(FRAM_CS, LOW);
    sendByte(0x06);
    digitalWrite(FRAM_CS, HIGH);
    delayMicroseconds(1);
    digitalWrite(FRAM_CS, LOW);
    sendByte(0x02);
    sendByte((addr >> 16) & 0xFF);
    sendByte((addr >> 8) & 0xFF);
    sendByte(addr & 0xFF);
    sendByte(data);
    digitalWrite(FRAM_CS, HIGH);
  }

  void write32(uint32_t addr, uint32_t data) {
    write8(addr, data & 0xFF);
    write8(addr + 1, (data >> 8) & 0xFF);
    write8(addr + 2, (data >> 16) & 0xFF);
    write8(addr + 3, (data >> 24) & 0xFF);
  }

  uint32_t read32(uint32_t addr) {
    uint8_t b0 = read8(addr);
    uint8_t b1 = read8(addr + 1);
    uint8_t b2 = read8(addr + 2);
    uint8_t b3 = read8(addr + 3);
    return ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
  }
};

// ═══════════════ MANUAL FRAM MASTERY - PURE HARDWARE INTERFACE ═══════════════
ManualFRAM fram;

//=============================================================================
// CALCULATE FRAM USAGE
//=============================================================================
uint32_t calculateFRAMUsage() {
  const uint32_t fixedCfg = 0x0200;                                             // small config near start
  const uint32_t featureCfg = 0x0100;                                           // feature flags etc.
  const uint32_t posLog = POSITION_SLOT_SIZE * (uint32_t)MAX_POSITION_HISTORY;  // 32000
  const uint32_t stateLog = STATE_RECORD_SIZE * (uint32_t)MAX_STATE_RECORDS;    // 3200
  uint32_t slots = (uint32_t)framManager.programCount * (uint32_t)FRAM_SLOT_SIZE;
  return fixedCfg + featureCfg + posLog + stateLog + slots;
}

//=============================================================================
// PERFORM FRAM CLEANUP
//=============================================================================
void smartFramManagement() {
  if (framManager.usagePercent >= 82) {
    Serial.println("FRAM 82% - Starting cleanup");

    // Step 1: Reduce position logging frequency (keeps recent data)
    if (positionLogIndex > 500) {
      // Keep only last 500 position entries, clear older ones
      for (int i = 0; i < 500; i++) {
        uint32_t oldAddr = FRAM_POSITION_HISTORY_BASE + (i * POSITION_SLOT_SIZE);
        for (int j = 0; j < POSITION_SLOT_SIZE; j++) {
          fram.write8(oldAddr + j, 0);
        }
      }
      positionLogIndex = 500;  // Reset index
    }

    // Step 2: Keep last 100 state records, clear older ones
    if (stateLogIndex > 100) {
      for (int i = 0; i < 100; i++) {
        uint32_t oldAddr = FRAM_STATE_LOG_BASE + (i * STATE_RECORD_SIZE);
        for (int j = 0; j < STATE_RECORD_SIZE; j++) {
          fram.write8(oldAddr + j, 0);
        }
      }
      stateLogIndex = 100;
    }

    // Step 3: Only if still over capacity, remove oldest 5 program slots
    framManager.currentUsage = calculateFRAMUsage();
    framManager.usagePercent = (framManager.currentUsage * 100) / FRAM_TOTAL_SIZE;

    if (framManager.usagePercent > 80 && framManager.programCount > 20) {
      // Remove 5 oldest program slots (your existing cleanup)
      smartFramManagement();
    }

    Serial.print("Cleanup complete. Usage now: ");
    Serial.print(framManager.usagePercent);
    Serial.println("%");
  }
}

//=============================================================================
// FRAM health monitoring
//=============================================================================
void monitorFRAMHealth() {
  static unsigned long lastHealthCheck = 0;
  static uint16_t lastSaveCount = 0;

  // Check FRAM health every 30 seconds
  if (millis() - lastHealthCheck > 30000) {
    lastHealthCheck = millis();

    // Check if save count is increasing abnormally fast
    uint16_t savesSinceLastCheck = framManager.saveCount - lastSaveCount;
    if (savesSinceLastCheck > 100) {
      Serial.println("WARNING: Excessive FRAM writes detected");
      Serial.print("Saves in last 30 seconds: ");
      Serial.println(savesSinceLastCheck);
    }

    lastSaveCount = framManager.saveCount;

    // Periodic usage report
    if (framManager.saveCount % 50 == 0) {
      Serial.print("FRAM health check - Save #");
      Serial.print(framManager.saveCount);
      Serial.print(", Usage: ");
      Serial.print(framManager.usagePercent);
      Serial.println("%");
    }
  }
}

//=============================================================================
// TRACK FRAM USAGE
//=============================================================================
void trackFRAMUsage() {
  framManager.currentUsage = calculateFRAMUsage();
  framManager.usagePercent =
    (uint8_t)((framManager.currentUsage * 100UL) / (uint32_t)FRAM_TOTAL_SIZE);
  framManager.saveCount++;
  framManager.lastSaveTime = millis();

  Serial.print("FRAM usage: ");
  Serial.print(framManager.currentUsage);
  Serial.print(" bytes (");
  Serial.print(framManager.usagePercent);
  Serial.print("%) - Programs: ");
  Serial.print(framManager.programCount);
  Serial.print("/");
  Serial.print(FRAM_PROGRAM_SLOTS);
  Serial.print(" - Save #");
  Serial.println(framManager.saveCount);

  if (framManager.currentUsage > (FRAM_MAX_USAGE - FRAM_TOLERANCE)) {
    framManager.needsCleanup = true;
    Serial.println("WARNING: FRAM approaching 80% capacity - cleanup scheduled");
  }
  if (framManager.currentUsage > (FRAM_MAX_USAGE + FRAM_TOLERANCE)) {
    Serial.println("CRITICAL: FRAM 82% capacity reached - performing emergency cleanup");
    smartFramManagement();
  }
}

// ═══════════════ TRIPLE DISPLAY TRINITY - VISUAL COMMAND CENTER ═══════════════
Adafruit_AlphaNum4 display1 = Adafruit_AlphaNum4();  // 0x70 - SH/CP
Adafruit_AlphaNum4 display2 = Adafruit_AlphaNum4();  // 0x72 - RC/VS
Adafruit_AlphaNum4 display3 = Adafruit_AlphaNum4();  // 0x74 - AW/AH

// ═══════════════ MOTION CONTROL NEXUS - STEPPER COMMAND PINS ═══════════════
#define Y_LEFT 10
#define Y_RIGHT 11
#define Z_DOWN 41
#define Z_UP 40
#define Y_STEP 26
#define Y_DIR 27
#define Z_STEP 31  // ═══════════════ FROM YOUR WORKING CODE - WAS INCORRECTLY CHANGED TO 24 ═══════════════//
#define Z_DIR 33

// ═══════════════ HUMAN INTERFACE PORTAL - BUTTON COMMAND PINS ═══════════════
#define PROGRAM_BTN 28
#define LOCKIN_BTN 29
#define START_BTN 30
#define SPEED_BTN 32

// ═══════════════ CHROMATIC FEEDBACK SYSTEM - LED STATUS RINGS ═══════════════
#define PROGRAM_LED_PIN 37
#define LOCKIN_LED_PIN 38
#define START_LED_PIN 36
#define SPEED_LED_PIN 39
#define LED_COUNT 1
#define BRIGHTNESS 100

// ═══════════════ QUADRUPLE ENCODER SYMPHONY - PRECISION INPUT MATRIX ═══════════════
#define ENC1_CLK 2
#define ENC1_DT 3
#define ENC1_SW 4
#define ENC2_CLK 5
#define ENC2_DT 6
#define ENC2_SW 7
#define ENC3_CLK 8
#define ENC3_DT 9
#define ENC3_SW 14
#define ENC4_CLK 15
#define ENC4_DT 16
#define ENC4_SW 17

// ═══════════════ MOTION DYNAMICS - RAMPING ELEGANCE CONSTANTS ═══════════════
#define MIN_STEP_DELAY 200      // µs (steady floor used in maps)
#define MAX_STEP_DELAY 1000     // µs (ramp start)
#define RAMP_UP_DURATION 400    // ms
#define RAMP_DOWN_DURATION 100  // ms
// NOTE: MOTOR_MIN_STEP_INTERVAL was conflicting; remove if unused.

// ═══════════════ CATHEDRAL DIMENSIONS - THE SACRED WORKSPACE ═══════════════
#define TOTAL_Y_TRAVEL_INCHES 66
#define TOTAL_Z_TRAVEL_INCHES 36

// ═══════════════ Y-AXIS CALIBRATION ═══════════════
constexpr float Y_STEPS_PER_INCH = 134.86f;  // from 20" test
static_assert(Y_STEPS_PER_INCH > 50.0f && Y_STEPS_PER_INCH < 1000.0f, "Y_STEPS_PER_INCH sanity");
constexpr long MAX_SAFE_Y_TRAVEL = long(TOTAL_Y_TRAVEL_INCHES * Y_STEPS_PER_INCH + 0.5f);

// ═══════════════ Z-AXIS MECHANICAL TRUTH ═══════════════
#define Z_MICROSTEPS_PER_REV 8000.0f          // DIP: ON ON OFF OFF
#define Z_GEAR_RATIO 10.0f                    // motor revs per pinion rev (10:1 reduction)
#define Z_PINION_TRAVEL_PER_REV_IN 3.0f       // inches per pinion rev
constexpr float Z_STEPS_PER_INCH = 32520.0f;  // Empirically calibrated
static_assert(Z_STEPS_PER_INCH > 1000.0f && Z_STEPS_PER_INCH < 50000.0f, "Z_STEPS_PER_INCH sanity");

constexpr long MAX_SAFE_Z_TRAVEL = long(TOTAL_Z_TRAVEL_INCHES * Z_STEPS_PER_INCH + 0.5f);
constexpr float Z_BACKOFF_INCH = 0.40f;  // change any time
constexpr long Z_BACKOFF_STEPS = long(Z_BACKOFF_INCH * Z_STEPS_PER_INCH + 0.5f);
constexpr long SAFE_HOME_OFFSET = long(1.0f * Z_STEPS_PER_INCH + 0.5f);

// ═══════════════ Y-AXIS MOTOR CONSTANTS ═══════════════
#define Y_FAST_DELAY 600        // µs (belt jog)
#define Y_PRECISION_DELAY 1200  // µs (fine positioning)

// ═══════════════ Z-AXIS MOTOR CONSTANTS ═══════════════
#define Z_NORMAL_SPEED_DELAY 10  // µs (true-home cruise). Watch stutter risk on heavy load.
#define Z_MACRO_SPEED_DELAY 100  // µs (macro jog)
#define Z_PULSE_WIDTH 15         // µs was 20
#define Y_PULSE_WIDTH 20         // was Z_PULSE_WIDTH (20)

// ═══════════════ SPRAY ACTIVATION NEXUS - SOLENOID COMMAND ═══════════════
// ─────────── UART1 NOTE (pin 0 / Serial1 RX) ───────────
// We intentionally do NOT use Serial1 in this project.
// Pin 0 is repurposed as SOLENOID_PIN (active-LOW) and remains GPIO.
// If Serial1.begin(...) is ever enabled, reassign SOLENOID_PIN to avoid UART1 conflicts.
#define SOLENOID_PIN 0

// ═══════════════ INTELLIGENT ENDSTOP PINS - DYNAMIC PROXIMITY SENSORS ═══════════════
//>#define Y_SENSING_ENDSTOP 1   // Y-axis sensing endstop (~6" from TRUE HOME)
#define Z_TOP_ENDSTOP 12  // TRUE HOME Z (physical limit)
//>#define Z_SENSING_ENDSTOP 34  // Z-axis sensing endstop (~6" from TRUE HOME)
// Only honor Z_SENSING when explicitly enabled (e.g., during homing).
//>static bool allowZSensingDuringMoves = false;
#define Y_RIGHT_ENDSTOP 35  // TRUE HOME Y (physical limit)

// Enables computing NEW HOME from parameters when LOCK is pressed at TRUE HOME
#ifndef INTELLIGENT_TRUE_HOME
#define INTELLIGENT_TRUE_HOME 1  // set to 0 to keep params-only behavior at TRUE HOME
#endif

// ---- Z Pulse Audit (minimal overhead) ----
static volatile unsigned long ZPulseTotal = 0;

// ═══════════════ NEW HOME at START ═══════════════
static bool movedToNewHomeThisCycle = false;

// ═══════════════ CLEAN ENDSTOP STATE MANAGEMENT ═══════════════
struct EndstopState {
  //>bool sensingEndstopsEnabled = false;  // Only active during TRUE HOME
  //>bool ySensingTriggered = false;       // Track if Y sensing triggered this cycle
  //>bool zSensingTriggered = false;       // Track if Z sensing triggered this cycle
  bool isHomingInProgress = false;  // TRUE HOME sequence active
} endstopState;

// Simple endstop disable system
static bool endstopsDisabled = false;
static long stepsSinceDisabled = 0;
const long Y_STEPS_TO_REENABLE = 500;   // Y needs only 500 steps
const long Z_STEPS_TO_REENABLE = 5000;  // Z keeps 5000 steps

// ═══════════════ EMERGENCY SAVE STATE TRACKING ═══════════════
struct EmergencySystem {
  bool powerLossDetected = false;
  bool emergencySaveComplete = false;
  unsigned long emergencyStartTime = 0;
  unsigned long lastPowerCheck = 0;
  int powerReadings[5] = { 1023, 1023, 1023, 1023, 1023 };  // ═══════════════ Rolling average ═══════════════
  int readingIndex = 0;
} emergencySystem;

// ═══════════════ DYNAMIC ENDSTOP INTELLIGENCE SYSTEM ═══════════════
struct DynamicEndstopData {
  // Rolling average of last 10 measurements
  long yDistanceHistory[10] = { 0 };
  long zDistanceHistory[10] = { 0 };
  int historyIndex = 0;
  long yAverageDistance = 0;
  long zAverageDistance = 0;

  // Change detection
  long yLastMeasurement = 0;
  long zLastMeasurement = 0;
  bool significantChangeDetected = false;

  // Confidence tracking
  int consecutiveGoodMeasurements = 0;
  int totalHomingCycles = 0;
  bool isCalibrated = false;

  // Health monitoring
  float yDriftPercent = 0.0;
  float zDriftPercent = 0.0;
  bool anomalyDetected = false;
  // unsigned long lastHealthCheck = 0; // Patch 2 applied – Ghost variables from v15.2 removed (scrollDelay, lastHealthCheck, etc.)
} dynamicEndstops;

// ═══════════════ PROGRAM MEMORY PALACE - SPRAY PARAMETERS ═══════════════
struct ProgramData {
  int sprayHeightInches = 1;
  int coveragePercent = 0;   // Will be loaded from FRAM at boot
  int areaWidthInches = 1;   // Area width in WHOLE INCHES
  int areaHeightInches = 1;  // Area height in WHOLE INCHES
  int repeatCount = 1;
  int velocityScale = 0;
  bool isValid = false;
} programMemory;

// ═══════════════ BUFFER ZONE GUARDIANS - SAFETY BOUNDARIES ═══════════════
struct BufferZones {
  long rightBufferY = 0;      // 3" right buffer (was rightBufferSteps)
  long leftBufferY = 0;       // 3" left buffer (was leftBufferSteps)
  long sprayStartOffset = 0;  // 0.5" before spray zone
  long sprayEndOffset = 0;    // 0.5" after spray zone
  bool isValid = false;
} bufferZones;

// ═══════════════ SYSTEM VARIABLES - COMPLETE 4-ENCODER ARCHITECTURE ═══════════════
struct SystemState {
  // ═══════════════ Display 1 variables (Encoder 1) - WHOLE INCHES ═══════════════
  int sprayHeightInches = 1;
  int coveragePercent = 50;  // ═══════════════ CP00-CP95 in steps of 5 ═══════════════
  enum Encoder1Mode { SH_MODE,
                      CP_MODE } encoder1Mode = SH_MODE;

  // ═══════════════ Display 2 variables (Encoder 2) - ENHANCED ═══════════════
  int repeatCount = 1;
  int velocityScale = 0;
  enum Encoder2Mode { RC_MODE,
                      V_PLUS_MODE } encoder2Mode = RC_MODE;

  // ═══════════════ Display 3 variables (Encoders 3 AND 4) - WHOLE INCHES ═══════════════
  int areaWidth = 1;   // ═══════════════ WHOLE INCHES ═══════════════
  int areaHeight = 1;  // ═══════════════ WHOLE INCHES ═══════════════
  // bool enc3InWidthMode = true; // ═══════════════ true = AW mode, false = W- mode ═══════════════ // Patch 2 applied – Ghost variables from v15.2 removed (scrollDelay, lastHealthCheck, etc.)
  // bool enc4InHeightMode = true; // ═══════════════ true = AH mode, false = H- mode ═══════════════ // Patch 2 applied – Ghost variables from v15.2 removed (scrollDelay, lastHealthCheck, etc.)
  bool showingWidth = true;  // ═══════════════ true = showing Width (AW/W-), false = showing Height (AH/H-) ═══════════════
} state;

// ═══════════════ BUTTON DEBOUNCERS - RELIABLE INPUT PROCESSING ═══════════════
Bounce programDebouncer = Bounce();
Bounce lockinDebouncer = Bounce();
Bounce startDebouncer = Bounce();
Bounce speedDebouncer = Bounce();

// ═══════════════ CHROMATIC FEEDBACK RINGS - VISUAL STATUS SYSTEM ═══════════════
Adafruit_NeoPixel programRing(LED_COUNT, PROGRAM_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lockinRing(LED_COUNT, LOCKIN_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel startRing(LED_COUNT, START_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel speedRing(LED_COUNT, SPEED_LED_PIN, NEO_GRB + NEO_KHZ800);

// ═══════════════ ENCODER STATES FOR ALL 4 ENCODERS ═══════════════
int lastClk1 = HIGH, lastClk2 = HIGH, lastClk3 = HIGH, lastClk4 = HIGH;
bool buttonLatched1 = false, buttonLatched2 = false, buttonLatched3 = false, buttonLatched4 = false;
unsigned long lastDebounceTime1 = 0, lastDebounceTime2 = 0, lastDebounceTime3 = 0, lastDebounceTime4 = 0;
const unsigned long debounceDelay = 5;

// ═══════════════ PREVIOUS TOGGLE STATES - FROM YOUR WORKING CODE ═══════════════
bool zUpPrev = true, zDownPrev = true, yLeftPrev = true, yRightPrev = true;

// ═══════════════ RAMPING VARIABLES - GRACEFUL MOTION CONTROL ═══════════════
unsigned long rampStartTime = 0;
bool isRampingUp = false, isRampingDown = false, isActivelyRampingDown = false;
unsigned long lastMotorAction = 0;
unsigned long lastMillis = 0;
const unsigned long encoderSampleRate = 5;  // ═══════════════ FIXED: (was 1) ═══════════════

// ═══════════════ AXIS MUTUAL EXCLUSION FLAGS ═══════════════
bool yAxisActive = false;
bool zAxisActive = false;
unsigned long lastAxisSwitchTime = 0;
const unsigned long AXIS_SWITCH_DELAY = 100;  // ═══════════════ 0.1s delay between ALL axis movements ═══════════════

// ───────── Post-release deadtime gates (stops micro-bursts) ─────────
static unsigned long yBlockUntil = 0;
static unsigned long zBlockUntil = 0;
const unsigned long TOGGLE_RELEASE_DEADTIME_MS = 120;  // ~0.12s feels natural

// ═══════════════ SPATIAL CONSCIOUSNESS - THE VISION (TRUE HOME vs NEW HOME) ═══════════════
struct PositionData {
  long trueHomeY = 0;       // TRUE HOME Y (absolute origin)
  long trueHomeZ = 0;       // TRUE HOME Z (absolute origin)
  long newHomeOffsetY = 0;  // NEW HOME offset from TRUE HOME
  long newHomeOffsetZ = 0;  // NEW HOME offset from TRUE HOME
  long currentY = 0;        // Current absolute position Y
  long currentZ = 0;        // Current absolute position Z
} position;

// ═══════════════ TOGGLE POSITION TRACKING - ALWAYS ACTIVE (REGARDLESS OF PROGRAM MODE) ═══════════════
struct ToggleTracking {
  long lastRecordedY = 0;    // Always tracking current Y position
  long lastRecordedZ = 0;    // Always tracking current Z position
  bool hasMovement = false;  // Flag to indicate movement has occurred
} toggleTracker;

// ═══════════════ SYSTEM STATE FLAGS - LOGICAL SPEED NAMING ═══════════════
bool programState = false;
bool lockState = false;
bool startState = false;
bool macroSpeed = false;  // ═══════════════ FALSE = NORMAL (Red, with ramping), TRUE = MACRO (Purple, no ramping) ═══════════════
bool systemArmed = false;
// bool isHomingMode = false;                // ═══════════════ Context flag for dual-purpose endstop behavior ═══════════════ // Patch 2 applied – Ghost variables from v15.2 removed (scrollDelay, lastHealthCheck, etc.)
bool endstopEmergencyTriggered = false;  // ═══════════════ Emergency stop state flag ═══════════════
bool systemLocked = false;               // ═══════════════ When true, only START and LOCK buttons work ═══════════════
bool sensorsActive = false;              // Master control for sensing proximety seonsor use

// ═══════════════ SYSTEM MODE ORCHESTRATOR - STATE MANAGEMENT ═══════════════
enum SystemMode { MANUAL_JOG,
                  PROGRAMMING,
                  LOCKED,
                  EXECUTING };
SystemMode currentMode = MANUAL_JOG;

// ═══════════════ FUNCTION DECLARATIONS - SYSTEM CAPABILITIES ═══════════════
void updateDisplay();
void handleAllEncoders();
void pulseMotor(int pin);
//>void pulseZMotor(bool isUp);
void trackMotorMovement(int pin, bool direction);
void performTrueHomeSequence();
void scrollBootMessage();
void showErrorLEDs();
void updateAllLEDs();
void executeAutomatedSequence();
void performControlledMoveWithSpray(long targetY, bool useRamping, bool useSolenoid);
void performZDropWithSymmetry(int dropInches);
void performZDropWithSymmetry(float dropInches);
bool isSafeToMove(bool yRequested, bool zRequested);
void establishYAxisBuffers();
void saveBufferZonesToFRAM();
void loadBufferZonesFromFRAM();
//void updateToggleTracking();
void lockInProgramMode();
void moveToRecordedNewHome();
void stopScrollingMessage();
void handleStartButtonLogic();
void performYAxisInitialization(bool movingRight);
void startScrollingMessage();
void updateAllDisplays();
// ═══════════════ CLEANED: Removed useSolenoid parameter ═══════════════
void performControlledMoveToPosition(long targetY, bool useRamping);
// ═══════════════ FRAM clean up and helpers ═══════════════
void performFRAMCleanup();
void incrementProgramCount();
void resetProgramCount();
void reportFRAMStatus();
// ═══════════════ INTELLIGENT ENDSTOP SYSTEM DECLARATIONS ═══════════════
bool validateEndstopLogic();
void loadDynamicEndstopData();
void updateDynamicDistance(char axis, long newDistance);
void saveDynamicEndstopData();
void detectEndstopAnomalies();
void checkSensorDrift();
void reportEndstopHealth();

bool setupComplete = false;

// ═══════════════ TRUE HOME vs NEW HOME LOGIC (SPEC-COMPLIANT) ═══════════════
void saveTrueHomeToFRAM() {
  Serial.println("💾 Saving TRUE HOME to FRAM...");
  fram.write32(FRAM_TRUE_HOME_Y, position.trueHomeY);
  fram.write32(FRAM_TRUE_HOME_Z, position.trueHomeZ);

  Serial.print("📍 TRUE HOME saved: Y=");
  Serial.print(position.trueHomeY);
  Serial.print(", Z=");
  Serial.println(position.trueHomeZ);
}

void loadTrueHomeFromFRAM() {
  position.trueHomeY = fram.read32(FRAM_TRUE_HOME_Y);
  position.trueHomeZ = fram.read32(FRAM_TRUE_HOME_Z);

  Serial.print("📍 TRUE HOME (FRAM snapshot): Y=");
  Serial.print(position.trueHomeY);
  Serial.print(", Z=");
  Serial.println(position.trueHomeZ);
}

// ═══════════════ FRAM LOAD (READ-ONLY; NO FIRST-BOOT SEEDING) ═══════════════
void loadNewHomeOffsetFromFRAM() {
  Serial.println("📥 ═══════════════ ENHANCED FRAM LOAD WITH VERIFICATION ═══════════════");
  uint8_t validFlag = fram.read8(FRAM_VALID_FLAG);
  Serial.print("🔍 VALID FLAG CHECK: ");
  Serial.println(validFlag == 1 ? "VALID" : "INVALID");

  if (validFlag != 1) {
    programMemory.isValid = false;
    // ❌ CRITICAL FIX: Don't reset RAM offsets to zero - preserve any existing values
    Serial.println("ℹ️ FRAM invalid. No data loaded from FRAM.");
    Serial.println("   LOCK away from TRUE HOME to create NEW HOME & params.");
    Serial.println("📥 ═══════════════ FRAM LOAD COMPLETE ═══════════════");
    return;
  }

  position.newHomeOffsetY = fram.read32(FRAM_NEW_HOME_Y_OFFSET);
  position.newHomeOffsetZ = fram.read32(FRAM_NEW_HOME_Z_OFFSET);

  state.sprayHeightInches = fram.read8(FRAM_PROGRAM_DATA + 0);
  state.coveragePercent = fram.read8(FRAM_PROGRAM_DATA + 1);
  state.areaWidth = fram.read8(FRAM_PROGRAM_DATA + 2);
  state.areaHeight = fram.read8(FRAM_PROGRAM_DATA + 3);
  state.velocityScale = fram.read8(FRAM_PROGRAM_DATA + 4);
  state.repeatCount = fram.read8(FRAM_PROGRAM_DATA + 5);

  programMemory.sprayHeightInches = state.sprayHeightInches;
  programMemory.coveragePercent = state.coveragePercent;
  programMemory.areaWidthInches = state.areaWidth;
  programMemory.areaHeightInches = state.areaHeight;
  programMemory.repeatCount = state.repeatCount;
  programMemory.velocityScale = state.velocityScale;
  programMemory.isValid = true;

  long absY = position.trueHomeY + position.newHomeOffsetY;
  long absZ = position.trueHomeZ + position.newHomeOffsetZ;

  Serial.print("📥 NEW HOME OFFSETS: dY=");
  Serial.print(position.newHomeOffsetY);
  Serial.print(" dZ=");
  Serial.println(position.newHomeOffsetZ);
  Serial.print("🎯 ABS NEW HOME: Y=");
  Serial.print(absY);
  Serial.print(" Z=");
  Serial.println(absZ);
  Serial.print("🎯 CALCULATED ABSOLUTE NEW HOME: Y=");
  Serial.print(absY);
  Serial.print(", Z=");
  Serial.println(absZ);
  Serial.println("📥 LOADED BASELINE PARAMETERS: SH=" + String(state.sprayHeightInches) + ", CP=" + String(state.coveragePercent) + ", AW=" + String(state.areaWidth) + ", AH=" + String(state.areaHeight) + ", V+=" + String(state.velocityScale) + ", RC=" + String(state.repeatCount));
  Serial.println("✅ FRAM LOAD SUCCESSFUL - All data restored");
  Serial.println("📥 ═══════════════ FRAM LOAD VERIFICATION COMPLETE ═══════════════");
}

//=============================================================================
// INCREMENT PROGRAM COUNT
//=============================================================================
void incrementProgramCount() {
  framManager.programCount++;
  if (framManager.programCount > FRAM_PROGRAM_SLOTS) {
    framManager.programCount = FRAM_PROGRAM_SLOTS;
  }
  trackFRAMUsage();
}

//=============================================================================
// RESET PROGRAM COUNT
//=============================================================================
void resetProgramCount() {
  framManager.programCount = 0;
  framManager.oldestSlot = 0;
  framManager.needsCleanup = false;
  Serial.println("FRAM program count reset");
}

// ═══════════════ FRAM INTEGRITY VALIDATION ═══════════════
void validateFRAMIntegrity() {
  Serial.println("🔍 ═══════════════ FRAM INTEGRITY CHECK ═══════════════");

  uint8_t validFlag = fram.read8(FRAM_VALID_FLAG);
  Serial.print("Valid flag: ");
  Serial.println(validFlag);

  if (validFlag == 1) {
    uint8_t aw = fram.read8(FRAM_BASELINE_PARAMETERS + 2);
    uint8_t ah = fram.read8(FRAM_BASELINE_PARAMETERS + 3);

    if (aw == 0 || aw > 100 || ah == 0 || ah > 100) {
      Serial.println("❌ BASELINE PARAMETERS CORRUPTED");
      Serial.print("   AW: ");
      Serial.println(aw);
      Serial.print("   AH: ");
      Serial.println(ah);

      fram.write8(FRAM_VALID_FLAG, 0);
      Serial.println("🔧 FRAM marked invalid - reprogram required");
    } else {
      Serial.println("✅ FRAM integrity OK");
    }
  } else {
    Serial.println("ℹ️ FRAM not programmed (valid flag = 0)");
  }
  Serial.println("🔍 ═══════════════ INTEGRITY CHECK COMPLETE ═══════════════");
}

//=============================================================================
// FRAM POSITION LOGGING SYSTEM - SMART FREQUENCY LOGGING
//=============================================================================
void logPositionToFRAM(uint8_t operation) {
#if POSITION_LOG_ENABLED
  uint32_t address = FRAM_POSITION_HISTORY_BASE + (positionLogIndex * POSITION_SLOT_SIZE);

  // Calculate integrity checksum
  uint8_t checksum = (uint8_t)(position.currentY + position.currentZ + toggleTracker.lastRecordedY + toggleTracker.lastRecordedZ) & 0xFF;

  // Write position record to FRAM
  fram.write32(address + 0, millis());                      // Timestamp
  fram.write32(address + 4, position.currentY);             // Current Y position
  fram.write32(address + 8, position.currentZ);             // Current Z position
  fram.write32(address + 12, toggleTracker.lastRecordedY);  // Toggle Y tracker
  fram.write32(address + 16, toggleTracker.lastRecordedZ);  // Toggle Z tracker
  fram.write8(address + 20, operation);                     // Operation code
  fram.write8(address + 21, checksum);                      // Integrity check
  // Bytes 22-31 remain as expansion space

  // Advance circular buffer index
  positionLogIndex = (positionLogIndex + 1) % MAX_POSITION_HISTORY;
#endif
}

// DRIFT DETECTION & CORRECTION GUARDIAN
void detectAndCorrectDrift() {
  long yDrift = abs(position.currentY - toggleTracker.lastRecordedY);
  long zDrift = abs(position.currentZ - toggleTracker.lastRecordedZ);

  if (yDrift > 10 || zDrift > 10) {
    Serial.println("POSITION DRIFT DETECTED - CORRECTING FROM FRAM MEMORY");
    Serial.print("   Y drift detected: ");
    Serial.print(yDrift);
    Serial.println(" steps");
    Serial.print("   Z drift detected: ");
    Serial.print(zDrift);
    Serial.println(" steps");

    // Force synchronization - motor position is truth
    toggleTracker.lastRecordedY = position.currentY;
    toggleTracker.lastRecordedZ = position.currentZ;

    // Log the correction for diagnostic purposes
    logPositionToFRAM(POS_OP_CORRECTION);
    Serial.println("Position sync restored - logged to FRAM");
  }
}

//=============================================================================
// FRAM STATE MANAGEMENT SYSTEM - THE LOGICAL GUARDIAN
//=============================================================================
void logSystemState(const char* trigger) {
  uint32_t address = FRAM_STATE_LOG_BASE + (stateLogIndex * STATE_RECORD_SIZE);

  // Write comprehensive state record
  fram.write32(address, millis());                 // Timestamp
  fram.write8(address + 4, programState ? 1 : 0);  // Program mode
  fram.write8(address + 5, lockState ? 1 : 0);     // Lock state
  fram.write8(address + 6, systemLocked ? 1 : 0);  // System locked
  fram.write8(address + 7, systemArmed ? 1 : 0);   // Armed state
  fram.write8(address + 8, (uint8_t)currentMode);  // Current mode
  fram.write8(address + 9, 0xAA);                  // Valid record marker

  // Advance circular buffer
  stateLogIndex = (stateLogIndex + 1) % MAX_STATE_RECORDS;

  Serial.print("STATE LOGGED: ");
  Serial.println(trigger);
}

// STATE VALIDATION & CONFLICT RESOLUTION
void validateSystemState() {
  bool changed;
  uint8_t passes = 0;
  do {
    changed = false;
    if (systemArmed && !lockState) {
      systemArmed = false;
      changed = true;
    }
    if (systemLocked && currentMode == MANUAL_JOG) {
      currentMode = LOCKED;
      changed = true;
    }
    if (programState && systemLocked) {
      programState = false;
      changed = true;
    }
    if (startState && (!lockState || !programMemory.isValid)) {
      startState = false;
      changed = true;
    }
  } while (changed && ++passes < 3);
  if (passes) {
    updateAllLEDs();
    logSystemState("AUTO_STATE_CORRECTION");
  }
}

// ═══════════════ Add after line 85 (after #define constants ═══════════════
enum DebugLevel { DEBUG_NONE,
                  DEBUG_BASIC,
                  DEBUG_VERBOSE };
DebugLevel currentDebugLevel = DEBUG_VERBOSE;

// Minimal Feature Flags System
#define FRAM_FEATURE_FLAGS_BASE 0x300

enum FeatureFlag {
  FEATURE_BUFFER_PERSISTENCE = 0,
  FEATURE_LED_BREATHING = 1,
  FEATURE_ENCODER_DEBOUNCE = 2,
  FEATURE_SOLENOID_GATING = 3
};

struct FeatureFlagManager {
  uint8_t flags = 0b1111;  // All 4 enabled by default

  bool isEnabled(FeatureFlag flag) {
    return (flags & (1 << flag)) != 0;
  }

  void saveToFRAM() {
    fram.write8(FRAM_FEATURE_FLAGS_BASE, flags);
    fram.write8(FRAM_FEATURE_FLAGS_BASE + 1, flags ^ 0xAA);  // Simple checksum
  }

  void loadFromFRAM() {
    uint8_t stored = fram.read8(FRAM_FEATURE_FLAGS_BASE);
    uint8_t check = fram.read8(FRAM_FEATURE_FLAGS_BASE + 1);
    if ((stored ^ 0xAA) == check) {
      flags = stored;
    } else {
      saveToFRAM();
    }
  }
} featureFlags;

void debugPrint(String message, DebugLevel level = DEBUG_BASIC) {
  if (currentDebugLevel >= level) {
    Serial.println(message);
  }
}

//=============================================================================
// REPORT FRAM STATUS - ENHANCED VERSION
//=============================================================================
void reportFRAMStatus() {
  Serial.println("===============================================");
  Serial.println("FRAM STATUS REPORT - Job Security v18.3");
  Serial.println("===============================================");
  Serial.print("FRAM chip: MB85RS4MT (512KB) - Total size: ");
  Serial.print(FRAM_TOTAL_SIZE);
  Serial.println(" bytes");
  Serial.print("Max safe usage (80%): ");
  Serial.print(FRAM_MAX_USAGE);
  Serial.println(" bytes");
  Serial.print("Current usage: ");
  Serial.print(framManager.currentUsage);
  Serial.print(" bytes (");
  Serial.print(framManager.usagePercent);
  Serial.println("%)");
  Serial.print("Program slots: ");
  Serial.print(framManager.programCount);
  Serial.print(" of ");
  Serial.println(FRAM_PROGRAM_SLOTS);
  Serial.print("Total saves: ");
  Serial.println(framManager.saveCount);
  Serial.print("Last save: ");
  if (framManager.lastSaveTime > 0) {
    Serial.print((millis() - framManager.lastSaveTime) / 1000);
    Serial.println(" seconds ago");
  } else {
    Serial.println("Never");
  }

  // Show actual data locations
  Serial.println("Data locations:");
  Serial.print("  NEW HOME Y: 0x");
  Serial.println(FRAM_NEW_HOME_Y_OFFSET, HEX);
  Serial.print("  NEW HOME Z: 0x");
  Serial.println(FRAM_NEW_HOME_Z_OFFSET, HEX);
  Serial.print("  Program Data: 0x");
  Serial.println(FRAM_PROGRAM_DATA, HEX);
  Serial.print("  Valid Flag: 0x");
  Serial.println(FRAM_VALID_FLAG, HEX);

  // Health assessment
  if (framManager.usagePercent < 10) {
    Serial.println("Status: EXCELLENT - Normal operation");
  } else if (framManager.usagePercent < 25) {
    Serial.println("Status: GOOD - Monitor for unusual growth");
  } else if (framManager.usagePercent < 50) {
    Serial.println("Status: CAUTION - Investigate high usage");
  } else {
    Serial.println("Status: CRITICAL - Immediate attention required");
  }

  Serial.println("===============================================");
}

// ═══════════════ ENHANCED FRAM SAVE (DUAL-MODE INTELLIGENT) ═══════════════
void saveNewHomeOffsetToFRAM() {
  Serial.println("💾 SIMPLE SAVE - No calculations, just current position");

  // Calculate offsets from current position
  long newOffsetY = position.currentY - position.trueHomeY;
  long newOffsetZ = position.currentZ - position.trueHomeZ;

  Serial.print("Saving offsets: Y=");
  Serial.print(newOffsetY);
  Serial.print(" Z=");
  Serial.println(newOffsetZ);

  // Save to FRAM
  fram.write32(FRAM_NEW_HOME_Y_OFFSET, newOffsetY);
  fram.write32(FRAM_NEW_HOME_Z_OFFSET, newOffsetZ);
  fram.write8(FRAM_VALID_FLAG, 1);

  // Update RAM
  position.newHomeOffsetY = newOffsetY;
  position.newHomeOffsetZ = newOffsetZ;

  Serial.println("✅ Simple save complete");
}

//>  Serial.println("💾 ═══════════════ FRAM SAVE (DUAL-MODE) ═══════════════");
//>  Serial.print("DEBUG: Current position Y=");
//>  Serial.print(position.currentY);
//>  Serial.print(" Z=");
//>  Serial.println(position.currentZ);
//>  Serial.print("DEBUG: TRUE HOME tolerance check: ");
//>  Serial.println(isAtTrueHome() ? "PASS" : "FAIL");

//>  // CASE A: At TRUE HOME → intelligent parameter handling
//>  if (isAtTrueHome()) {
//>    Serial.println("🔒 LOCK @ TRUE HOME → Analyzing parameter changes");

//>    // Get current stored parameters
//>    uint8_t validFlag = fram.read8(FRAM_VALID_FLAG);
//>    bool hasValidBaseline = (validFlag == 1);

//>    if (!hasValidBaseline) {
//>      // First-time programming
//>      Serial.println("🆕 First-time programming - establishing baseline");
//>      long currentNewHomeY = fram.read32(FRAM_NEW_HOME_Y_OFFSET);
//>      long currentNewHomeZ = fram.read32(FRAM_NEW_HOME_Z_OFFSET);
//>      commitOffsetsAndParamsAtomic(currentNewHomeY, currentNewHomeZ);
//>      return;
//>    }

//>    // Read current baseline parameters
//>    uint8_t oldSH = fram.read8(FRAM_BASELINE_PARAMETERS + 0);
//>    uint8_t oldCP = fram.read8(FRAM_BASELINE_PARAMETERS + 1);
//>    uint8_t oldAW = fram.read8(FRAM_BASELINE_PARAMETERS + 2);
//>    uint8_t oldAH = fram.read8(FRAM_BASELINE_PARAMETERS + 3);
//>    uint8_t oldV = fram.read8(FRAM_BASELINE_PARAMETERS + 4);
//>    uint8_t oldRC = fram.read8(FRAM_BASELINE_PARAMETERS + 5);

//>    // Compare with new parameters
//>    uint8_t newSH = state.sprayHeightInches;
//>    uint8_t newCP = state.coveragePercent;
//>    uint8_t newAW = state.areaWidth;
//>    uint8_t newAH = state.areaHeight;
//>    uint8_t newV = state.velocityScale;
//>    uint8_t newRC = state.repeatCount;

//>    // Determine what changed
//>    bool awChanged = (oldAW != newAW);
//>    bool ahChanged = (oldAH != newAH);
//>    bool shChanged = (oldSH != newSH);
//>    bool cpChanged = (oldCP != newCP);
//>    bool vChanged = (oldV != newV);
//>    bool rcChanged = (oldRC != newRC);

//>    //>bool onlySubstrateChanged = (awChanged || ahChanged) && !shChanged && !cpChanged && !vChanged && !rcChanged;

//>    long currentNewHomeY = fram.read32(FRAM_NEW_HOME_Y_OFFSET);
//>    long currentNewHomeZ = fram.read32(FRAM_NEW_HOME_Z_OFFSET);

//>    {
//>      // ═══════════════ UNIFIED MODE: Calculate optimal NEW HOME for all parameter changes ═══════════════
//>      Serial.println("🎯 UNIFIED MODE: Calculating optimal NEW HOME for parameter set");

//>      // Show all changes
//>      if (awChanged) {
//>        Serial.print("   AW: ");
//>        Serial.print(oldAW);
//>        Serial.print(" → ");
//>        Serial.println(newAW);
//>      }
//>      if (ahChanged) {
//>        Serial.print("   AH: ");
//>        Serial.print(oldAH);
//>        Serial.print(" → ");
//>        Serial.println(newAH);
//>      }
//>      if (shChanged) {
//>        Serial.print("   SH: ");
//>        Serial.print(oldSH);
//>        Serial.print(" → ");
//>        Serial.println(newSH);
//>      }
//>      if (cpChanged) {
//>        Serial.print("   CP: ");
//>        Serial.print(oldCP);
//>        Serial.print("% → ");
//>        Serial.print(newCP);
//>        Serial.println("%");
//>      }
//>      if (vChanged) {
//>        Serial.print("   V+: ");
//>        Serial.print(oldV);
//>        Serial.print(" → ");
//>        Serial.println(newV);
//>      }
//>      if (rcChanged) {
//>        Serial.print("   RC: ");
//>        Serial.print(oldRC);
//>        Serial.print(" → ");
//>        Serial.println(newRC);
//>      }

//>      // Calculate optimal NEW HOME based on substrate positioning requirements
//>      // NEW HOME should be at top-right corner of substrate for spray execution
//>      long optimalY = -(newAW * Y_STEPS_PER_INCH);  // LEFT from TRUE HOME by substrate width
//>      long optimalZ = -(newAH * Z_STEPS_PER_INCH);  // DOWN from TRUE HOME by substrate height

//>      // Apply SH offset - NEW HOME needs to account for initial SH drop positioning
//>      long shOffset = (newSH * Z_STEPS_PER_INCH) / 2;  // Half SH above substrate top
//>      optimalZ += shOffset;

//>      Serial.print("📍 Optimal NEW HOME calculation:");
//>      Serial.print("  Y=");
//>      Serial.print(optimalY);
//>      Serial.print(" (");
//>      Serial.print(newAW);
//>      Serial.println("\" LEFT)");
//>      Serial.print("  Z=");
//>      Serial.print(optimalZ);
//>      Serial.print(" (");
//>      Serial.print(newAH);
//>      Serial.print("\" DOWN + ");
//>      Serial.print(newSH / 2.0);
//>      Serial.println("\" SH offset)");

//>      Serial.print("📍 NEW HOME: (");
//>      Serial.print(currentNewHomeY);
//>      Serial.print(",");
//>      Serial.print(currentNewHomeZ);
//>      Serial.print(") → (");
//>      Serial.print(optimalY);
//>      Serial.print(",");
//>      Serial.print(optimalZ);
//>      Serial.println(")");

//>      commitOffsetsAndParamsAtomic(optimalY, optimalZ);
//>      return;  // ← ADD THIS LINE
//>    }
//>  }

/*
  // CASE B: Away from TRUE HOME → standard position-based save
  long sourceY = position.currentY;
  long sourceZ = position.currentZ;
  long newOffsetY = sourceY - position.trueHomeY;
  long newOffsetZ = sourceZ - position.trueHomeZ;

  // Debug output
  Serial.print("📊 SAVE INPUT — Current Position Y=");
  Serial.print(sourceY);
  Serial.print(" Z=");
  Serial.println(sourceZ);
  Serial.print("📊 NEW HOME OFFSETS — dY=");
  Serial.print(newOffsetY);
  Serial.print(" dZ=");
  Serial.println(newOffsetZ);

  uint8_t SHb = clampToByte(state.sprayHeightInches);
  uint8_t CPb = clampToByte(state.coveragePercent);
  uint8_t AWb = clampToByte(state.areaWidth);
  uint8_t AHb = clampToByte(state.areaHeight);
  uint8_t Vb = clampToByte(state.velocityScale);
  uint8_t Rb = clampToByte(state.repeatCount);

  // Atomic: mark invalid → write → verify → mark valid
  fram.write8(FRAM_VALID_FLAG, 0);

  fram.write32(FRAM_NEW_HOME_Y_OFFSET, newOffsetY);
  fram.write32(FRAM_NEW_HOME_Z_OFFSET, newOffsetZ);

  fram.write8(FRAM_PROGRAM_DATA + 0, SHb);
  fram.write8(FRAM_PROGRAM_DATA + 1, CPb);
  fram.write8(FRAM_PROGRAM_DATA + 2, AWb);
  fram.write8(FRAM_PROGRAM_DATA + 3, AHb);
  fram.write8(FRAM_PROGRAM_DATA + 4, Vb);
  fram.write8(FRAM_PROGRAM_DATA + 5, Rb);

  fram.write8(FRAM_BASELINE_PARAMETERS + 0, SHb);
  fram.write8(FRAM_BASELINE_PARAMETERS + 1, CPb);
  fram.write8(FRAM_BASELINE_PARAMETERS + 2, AWb);
  fram.write8(FRAM_BASELINE_PARAMETERS + 3, AHb);
  fram.write8(FRAM_BASELINE_PARAMETERS + 4, Vb);
  fram.write8(FRAM_BASELINE_PARAMETERS + 5, Rb);

  bool verified = false;
  for (int attempt = 0; attempt < 2; ++attempt) {
    long ry = fram.read32(FRAM_NEW_HOME_Y_OFFSET);
    long rz = fram.read32(FRAM_NEW_HOME_Z_OFFSET);
    uint8_t rSH = fram.read8(FRAM_PROGRAM_DATA + 0);
    uint8_t rCP = fram.read8(FRAM_PROGRAM_DATA + 1);
    uint8_t rAW = fram.read8(FRAM_PROGRAM_DATA + 2);
    uint8_t rAH = fram.read8(FRAM_PROGRAM_DATA + 3);
    uint8_t rV = fram.read8(FRAM_PROGRAM_DATA + 4);
    uint8_t rRC = fram.read8(FRAM_PROGRAM_DATA + 5);

    bool offsetsOK = (ry == newOffsetY) && (rz == newOffsetZ);
    bool paramsOK = (rSH == SHb && rCP == CPb && rAW == AWb && rAH == AHb && rV == Vb && rRC == Rb);

    if (offsetsOK && paramsOK) {
      verified = true;
      break;
    }
    Serial.println("⚠️ FRAM verify mismatch; retrying write...");
    if (!offsetsOK) {
      fram.write32(FRAM_NEW_HOME_Y_OFFSET, newOffsetY);
      fram.write32(FRAM_NEW_HOME_Z_OFFSET, newOffsetZ);
    }
    if (!paramsOK) {
      fram.write8(FRAM_PROGRAM_DATA + 0, SHb);
      fram.write8(FRAM_PROGRAM_DATA + 1, CPb);
      fram.write8(FRAM_PROGRAM_DATA + 2, AWb);
      fram.write8(FRAM_PROGRAM_DATA + 3, AHb);
      fram.write8(FRAM_PROGRAM_DATA + 4, Vb);
      fram.write8(FRAM_PROGRAM_DATA + 5, Rb);
      fram.write8(FRAM_BASELINE_PARAMETERS + 0, SHb);
      fram.write8(FRAM_BASELINE_PARAMETERS + 1, CPb);
      fram.write8(FRAM_BASELINE_PARAMETERS + 2, AWb);
      fram.write8(FRAM_BASELINE_PARAMETERS + 3, AHb);
      fram.write8(FRAM_BASELINE_PARAMETERS + 4, Vb);
      fram.write8(FRAM_BASELINE_PARAMETERS + 5, Rb);
    }
  }

  if (!verified) {
    Serial.println("❌ FRAM SAVE FAILED AFTER RETRY. VALID_FLAG remains 0.");
    return;
  }

  fram.write8(FRAM_VALID_FLAG, 1);

  position.newHomeOffsetY = newOffsetY;
  position.newHomeOffsetZ = newOffsetZ;
  programMemory.sprayHeightInches = SHb;
  programMemory.coveragePercent = CPb;
  programMemory.areaWidthInches = AWb;
  programMemory.areaHeightInches = AHb;
  programMemory.repeatCount = Rb;
  programMemory.velocityScale = Vb;
  programMemory.isValid = true;

  Serial.println("✅ FRAM SAVE COMPLETE (offsets + parameters) & VERIFIED");
  trackFRAMUsage();  // Check FRAM usage after every save

  // Log NEW HOME establishment to FRAM
  logPositionToFRAM(POS_OP_NEW_HOME);
  logSystemState("NEW_HOME_SAVED");
  Serial.println("NEW HOME settings preserved in FRAM cathedral");
*/

//>// ═══════════════ ENHANCED ATOMIC COMMIT WITH VALIDATION ═══════════════
//>/#if INTELLIGENT_TRUE_HOME
//>static void commitOffsetsAndParamsAtomic(int32_t newOffsetY, int32_t newOffsetZ) {
//>  uint8_t SHb = clampToByte(state.sprayHeightInches);
//>  uint8_t CPb = clampToByte(state.coveragePercent);
//>  uint8_t AWb = clampToByte(state.areaWidth);
//>  uint8_t AHb = clampToByte(state.areaHeight);
//>  uint8_t Vb = clampToByte(state.velocityScale);
//>  uint8_t Rb = clampToByte(state.repeatCount);
//>
//>  // Atomic transaction
//>  fram.write8(FRAM_VALID_FLAG, 0);
//>
//>  fram.write32(FRAM_NEW_HOME_Y_OFFSET, (uint32_t)newOffsetY);
//>  fram.write32(FRAM_NEW_HOME_Z_OFFSET, (uint32_t)newOffsetZ);
//>
//>  fram.write8(FRAM_PROGRAM_DATA + 0, SHb);
//>  fram.write8(FRAM_PROGRAM_DATA + 1, CPb);
//>  fram.write8(FRAM_PROGRAM_DATA + 2, AWb);
//>  fram.write8(FRAM_PROGRAM_DATA + 3, AHb);
//>  fram.write8(FRAM_PROGRAM_DATA + 4, Vb);
//>  fram.write8(FRAM_PROGRAM_DATA + 5, Rb);
//>
//>  fram.write8(FRAM_BASELINE_PARAMETERS + 0, SHb);
//>  fram.write8(FRAM_BASELINE_PARAMETERS + 1, CPb);
//>  fram.write8(FRAM_BASELINE_PARAMETERS + 2, AWb);
//>  fram.write8(FRAM_BASELINE_PARAMETERS + 3, AHb);
//>  fram.write8(FRAM_BASELINE_PARAMETERS + 4, Vb);
//>  fram.write8(FRAM_BASELINE_PARAMETERS + 5, Rb);
//>
//>  // Comprehensive verification
//>  int32_t ry = (int32_t)fram.read32(FRAM_NEW_HOME_Y_OFFSET);
//>  int32_t rz = (int32_t)fram.read32(FRAM_NEW_HOME_Z_OFFSET);
//>  uint8_t rSH = fram.read8(FRAM_PROGRAM_DATA + 0);
//>  uint8_t rCP = fram.read8(FRAM_PROGRAM_DATA + 1);
//>  uint8_t rAW = fram.read8(FRAM_PROGRAM_DATA + 2);
//>  uint8_t rAH = fram.read8(FRAM_PROGRAM_DATA + 3);
//>  uint8_t rV = fram.read8(FRAM_PROGRAM_DATA + 4);
//>  uint8_t rRC = fram.read8(FRAM_PROGRAM_DATA + 5);
//>
//>  bool ok = (ry == newOffsetY) && (rz == newOffsetZ) && (rSH == SHb && rCP == CPb && rAW == AWb && rAH == AHb && rV == Vb && rRC == Rb);
//>
//>  if (!ok) {
//>    Serial.println("❌ Atomic commit verification failed; leaving VALID=0");
//>    Serial.print("Expected Y=");
//>    Serial.print(newOffsetY);
//>    Serial.print(", got ");
//>    Serial.println(ry);
//>    Serial.print("Expected Z=");
//>    Serial.print(newOffsetZ);
//>    Serial.print(", got ");
//>    Serial.println(rz);
//>    return;
//>  }
//>
//>  fram.write8(FRAM_VALID_FLAG, 1);
//>
//>  // Update RAM to match FRAM
//>  position.newHomeOffsetY = newOffsetY;
//>  position.newHomeOffsetZ = newOffsetZ;
//>  programMemory.sprayHeightInches = SHb;
//>  programMemory.coveragePercent = CPb;
//>  programMemory.areaWidthInches = AWb;
//>  programMemory.areaHeightInches = AHb;
//>  programMemory.repeatCount = Rb;
//>  programMemory.velocityScale = Vb;
//>  programMemory.isValid = true;
//>
//>  Serial.println("✅ Intelligent TRUE HOME commit complete (offsets + params).");
//>}
//>/#endif

#ifndef INTELLIGENT_TRUE_HOME
#define INTELLIGENT_TRUE_HOME 1  // TRUE-HOME LOCK commits offsets from params
#endif

long getNewHomeAbsoluteY() {
  return position.trueHomeY + position.newHomeOffsetY;
}

long getNewHomeAbsoluteZ() {
  return position.trueHomeZ + position.newHomeOffsetZ;
}

// Treat within ~0.5" on Y and ~0.10" on Z as TRUE HOME for LOCK behavior
bool isAtTrueHome() {
  const long yTol = (long)(3.0f * Y_STEPS_PER_INCH + 0.5f);  // 3"
  const long zTol = (long)(3.0f * Z_STEPS_PER_INCH + 0.5f);  // 3"
  return (labs(position.currentY - position.trueHomeY) <= yTol) && (labs(position.currentZ - position.trueHomeZ) <= zTol);
}

bool isAtNewHome() {
  long newHomeY = getNewHomeAbsoluteY();
  long newHomeZ = getNewHomeAbsoluteZ();
  return (labs(position.currentY - newHomeY) < 10) && (labs(position.currentZ - newHomeZ) < 10);
}

// ═══════════════ PERIODIC SAVE FUNCTION - TELEMETRY ONLY (NO WRITES) ═══════════════
#ifndef TELEMETRY_PERIOD_MS
#define TELEMETRY_PERIOD_MS 10000UL  // 10s default; change once if you ever need to
#endif
#ifndef TELEMETRY_ENABLED
#define TELEMETRY_ENABLED 1  // set to 0 to compile-out telemetry entirely
#endif

void performPeriodicSave() {
#if TELEMETRY_ENABLED
  // Auto-mute when USB serial isn't connected (prevents pointless prints in the field)
  if (!Serial) return;

  static unsigned long lastLog = 0;
  static long lastY = 0x7FFFFFFF;
  static long lastZ = 0x7FFFFFFF;

  const unsigned long now = millis();
  // Wrap-safe period check
  if ((unsigned long)(now - lastLog) >= TELEMETRY_PERIOD_MS) {
    lastLog = now;  // advance the window first (avoids tight-loop retries if printing stalls)

    const long y = position.currentY;
    const long z = position.currentZ;

    // Only log when something actually changed
    if (y != lastY || z != lastZ) {
      lastY = y;
      lastZ = z;
      Serial.print(F("📝 Telemetry: Y="));
      Serial.print(y);
      Serial.print(F(", Z="));
      Serial.println(z);
    }
  }
#endif
}

void performZMoveWithTimeBasedRamping(long steps, bool isUp) {
  Serial.print("🎯 Z Ramp: ");
  Serial.print(steps);
  Serial.print(" steps, direction: ");
  Serial.println(isUp ? "UP" : "DOWN");

  digitalWrite(Z_DIR, isUp ? HIGH : LOW);

  const int startDelay = 10;
  const int endDelay = 1;
  const unsigned long rampTime = 500000UL;  // 0.5s ramp

  int currentDelay = startDelay;
  unsigned long startTime = micros();
  const int zSign = isUp ? 1 : -1;

  for (long i = 0; i < steps; i++) {
    if (!isUp && digitalRead(Z_TOP_ENDSTOP) == LOW) {
      Serial.println("🏠 Z TRUE HOME hit - STOPPING RAMP");
      break;
    }

    unsigned long elapsed = micros() - startTime;
    if (elapsed < rampTime) {
      float progress = (float)elapsed / (float)rampTime;  // 0→1
      currentDelay = startDelay - (int)((startDelay - endDelay) * progress);
    } else {
      currentDelay = endDelay;
    }

    currentDelay = zSpeed(currentDelay);  // apply V+ scaling to Z

    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(Z_PULSE_WIDTH);
    digitalWrite(Z_STEP, LOW);
    ++ZPulseTotal;  // <— ADD here too
    delayMicroseconds(currentDelay);

    position.currentZ += zSign;
  }
  // Ensure clean axis lifecycle
  zAxisActive = false;
}

void lockInProgramMode() {
  Serial.println("🔧 PROGRAM MODE ENABLED - Locking in last known positions as NEW HOME");

  // Calculate NEW HOME offset from TRUE HOME using LAST KNOWN positions
  position.newHomeOffsetY = toggleTracker.lastRecordedY - position.trueHomeY;
  position.newHomeOffsetZ = toggleTracker.lastRecordedZ - position.trueHomeZ;

  // Use the enhanced save function instead of direct FRAM writes
  saveNewHomeOffsetToFRAM();

  Serial.print("💾 NEW HOME locked in at offset: Y=");
  Serial.print(position.newHomeOffsetY);
  Serial.print(", Z=");
  Serial.println(position.newHomeOffsetZ);
}

// CONSISTENT COATING DENSITY - FIXED FOR UNIFORM DARKNESS
int calculateFinalPassCount(int areaHeightInches, int sprayHeightInches, int coveragePercent) {
  if (sprayHeightInches <= 0) return 1;
  if (coveragePercent >= 100) coveragePercent = 95;

  float overlapFraction = coveragePercent / 100.0;
  float effectiveStepSize = sprayHeightInches * (1.0 - overlapFraction);
  if (effectiveStepSize <= 0.01) effectiveStepSize = 0.01;

  // UNIFORM DARKNESS: Exact passes per inch, no rounding bias
  float passesPerInch = 1.0 / effectiveStepSize;
  int calculatedPasses = max(1, (int)round(areaHeightInches * passesPerInch));

  return calculatedPasses;
}

// ═══════════════ UPDATED RECORDED NEW HOME MOVEMENT (uses live state.AH & AW) ═══════════════
void moveToRecordedNewHome() {
  Serial.println("🏠 Moving to recorded NEW HOME position...");

  // ✅ DISABLE Z/Y sensing during the jump to NEW HOME
  //>bool originalSensingState = allowZSensingDuringMoves;
  //>allowZSensingDuringMoves = false;
  //>Serial.println("🔇 Sensing endstops DISABLED for NEW HOME movement");

  // Base absolute NEW HOME from committed offsets
  const long baseNewHomeY = position.trueHomeY + position.newHomeOffsetY;
  const long baseNewHomeZ = position.trueHomeZ + position.newHomeOffsetZ;

  // ═══════════════ Track FRAM usage for continuous position monitoring ═══════════════
  trackFRAMUsage();

  // ═══════════════ Just track - saving happens when LOCK button is pressed ═══════════════
  Serial.print("📍 Toggle position tracked: Y=");
  Serial.print(position.currentY);
  Serial.print(", Z=");
  Serial.println(position.currentZ);

  // Baseline AH (when NEW HOME was last committed)
  int baselineAH = fram.read8(FRAM_BASELINE_PARAMETERS + 3);
  if (baselineAH == 0xFF || baselineAH == 0) baselineAH = state.areaHeight;  // fallback

  // Baseline AW (when NEW HOME was last committed)
  int baselineAW = fram.read8(FRAM_BASELINE_PARAMETERS + 2);
  if (baselineAW == 0xFF || baselineAW == 0) baselineAW = state.areaWidth;  // fallback

  // ❗ Use LIVE state (RAM) so TRUE-HOME programming applies immediately
  const long originalAHSteps = (long)baselineAH * Z_STEPS_PER_INCH;
  const long currentAHSteps = (long)state.areaHeight * Z_STEPS_PER_INCH;
  const long originalAWSteps = (long)baselineAW * Y_STEPS_PER_INCH;
  const long currentAWSteps = (long)state.areaWidth * Y_STEPS_PER_INCH;

  // Z is positive-up. If you go AH10 -> AH05, current - original = -5" ⇒ Z DOWN
  const long ahCompensation = currentAHSteps - originalAHSteps;

  // Y is negative-left. If you go AW24 -> AW30, current - original = +6" ⇒ Y LEFT (more negative)
  const long awCompensation = currentAWSteps - originalAWSteps;

  // Apply both compensations
  const long targetY = baseNewHomeY + awCompensation;
  const long targetZ = baseNewHomeZ + ahCompensation;

  Serial.print("🎯 NEW HOME (effective): Y=");
  Serial.print(targetY);
  Serial.print(" Z=");
  Serial.println(targetZ);

  Serial.print("📊 Compensations applied: AW=");
  Serial.print(awCompensation);
  Serial.print(" steps, AH=");
  Serial.print(ahCompensation);
  Serial.println(" steps");

  // Move Z first, then Y (keeps your existing ramp helpers)
  long zStepsNeeded = targetZ - position.currentZ;
  if (labs(zStepsNeeded) > 10) {
    performZMoveWithTimeBasedRamping(labs(zStepsNeeded), zStepsNeeded > 0);
  }
  delay(AXIS_SWITCH_DELAY);

  long yStepsNeeded = targetY - position.currentY;
  if (labs(yStepsNeeded) > 10) {
    digitalWrite(Y_DIR, yStepsNeeded > 0 ? HIGH : LOW);
    for (long i = 0; i < labs(yStepsNeeded); i++) {
      digitalWrite(Y_STEP, HIGH);
      delayMicroseconds(600);
      digitalWrite(Y_STEP, LOW);
      delayMicroseconds(600);
      position.currentY += (yStepsNeeded > 0) ? 1 : -1;
    }
  }

  // Keep software + tracker in lockstep after Y return
  toggleTracker.lastRecordedY = position.currentY;
  toggleTracker.lastRecordedZ = position.currentZ;

  Serial.println("✅ NEW HOME reached with compensations applied");
}

// Shared compensation calculation for consistent coordinate system
long getCompensatedNewHomeY() {
  // Base NEW HOME from FRAM
  long baseNewHomeY = position.trueHomeY + position.newHomeOffsetY;

  // Get baseline parameters
  int baselineAW = fram.read8(FRAM_BASELINE_PARAMETERS + 2);
  if (baselineAW == 0xFF || baselineAW == 0) baselineAW = state.areaWidth;

  // Calculate compensation
  const long originalAWSteps = (long)baselineAW * Y_STEPS_PER_INCH;
  const long currentAWSteps = (long)state.areaWidth * Y_STEPS_PER_INCH;
  const long awCompensation = currentAWSteps - originalAWSteps;

  return baseNewHomeY + awCompensation;
}

long getCompensatedNewHomeZ() {
  // Base NEW HOME from FRAM
  long baseNewHomeZ = position.trueHomeZ + position.newHomeOffsetZ;

  // Get baseline parameters
  int baselineAH = fram.read8(FRAM_BASELINE_PARAMETERS + 3);
  if (baselineAH == 0xFF || baselineAH == 0) baselineAH = state.areaHeight;

  // Calculate compensation
  const long originalAHSteps = (long)baselineAH * Z_STEPS_PER_INCH;
  const long currentAHSteps = (long)state.areaHeight * Z_STEPS_PER_INCH;
  const long ahCompensation = currentAHSteps - originalAHSteps;

  return baseNewHomeZ + ahCompensation;
}

void stopScrollingMessage() {
  scrollMessage.isActive = false;
  updateDisplay();
}

void calculateNewHomeFromTrueHome() {
  // Calculate NEW HOME coordinates based on current parameters
  long newOffsetY = -(state.areaWidth * Y_STEPS_PER_INCH);   // LEFT from TRUE HOME
  long newOffsetZ = -(state.areaHeight * Z_STEPS_PER_INCH);  // DOWN from TRUE HOME

  // Apply SH positioning offset
  long shOffset = (state.sprayHeightInches * Z_STEPS_PER_INCH) / 2;
  newOffsetZ += shOffset;

  // Store the calculated coordinates
  position.newHomeOffsetY = newOffsetY;
  position.newHomeOffsetZ = newOffsetZ;

  // Store baseline parameters for future compensation
  fram.write8(FRAM_BASELINE_PARAMETERS + 2, state.areaWidth);
  fram.write8(FRAM_BASELINE_PARAMETERS + 3, state.areaHeight);
  fram.write8(FRAM_BASELINE_PARAMETERS + 4, state.sprayHeightInches);

  saveNewHomeOffsetToFRAM();
}

// ═══════════════ START BUTTON HANDLER (the nuclear option) ═══════════════
void handleStartButtonLogic() {
  Serial.println("START pressed - Simple repeater mode");

  simpleGoToSavedPosition();

  currentMode = EXECUTING;
  systemArmed = true;
  executeAutomatedSequence();

  currentMode = MANUAL_JOG;
  systemArmed = false;
  updateAllLEDs();
}

// ═══════════════ START BUTTON HANDLER (commit-at-TRUE-HOME model) ═══════════════
//>void handleStartButtonLogic() {
//>Serial.println("▶️ START pressed");

// 1) Hard guard: must have a valid FRAM program (NEW HOME + params)
//>if (fram.read8(FRAM_VALID_FLAG) != 1) {
//>Serial.println("🛑 START blocked: FRAM invalid. LOCK away from TRUE HOME to define NEW HOME & params.");
//>updateAllLEDs();
// Log button interaction and validate state
//>logSystemState("BUTTON_PRESSED");
//>validateSystemState();
//>return;
//>}

// 2) Optional guard: require lock engaged to run (toggle ON if you want this)
//>if (!lockState) {
//>Serial.println("🛑 START blocked: system is unlocked. Press LOCK first.");
//>updateAllLEDs();
// Log button interaction and validate state
//>logSystemState("BUTTON_PRESSED");
//>validateSystemState();
//>return;
//>}

// 3) Stop any boot/completion scrolling message on user action
//>if (scrollMessage.isActive) {
//>scrollMessage.isActive = false;
//>}

// 4) Make sure the solenoid is OFF before travel
//>digitalWrite(SOLENOID_PIN, HIGH);  // 5) (Optional) disable Z sensing during the reposition to NEW HOME to avoid nuisance trips
//>bool prevZSense = allowZSensingDuringMoves;
//>allowZSensingDuringMoves = false;

// If at TRUE HOME, recalculate NEW HOME from current parameters
//>float yInchesFromTrueHome = abs(position.currentY - position.trueHomeY) / Y_STEPS_PER_INCH;
//>float zInchesFromTrueHome = abs(position.currentZ - position.trueHomeZ) / Z_STEPS_PER_INCH;
//>bool atTrueHome = (yInchesFromTrueHome <= 1.0) && (zInchesFromTrueHome <= 1.0);

//>if (atTrueHome) {
//>Serial.println("📍 At TRUE HOME - recalculating NEW HOME from current parameters");
//>calculateNewHomeFromTrueHome();
//>}

// Always move to NEW HOME before executing
//>moveToRecordedNewHome();

// Check if we're already at NEW HOME (within tolerance) - using compensated coordinates
//>moveToRecordedNewHome();

// 7) Restore sensing state
//>allowZSensingDuringMoves = prevZSense;

// 8) Run the program (“dance”)
//>currentMode = EXECUTING;
//>systemArmed = true;
//>executeAutomatedSequence();

// ✅ Reset state after sequence completion
//>currentMode = MANUAL_JOG;  // <--- This is your valid idle/default mode
//>systemArmed = false;
//>updateAllLEDs();  // Refresh LED display
// Log button interaction and validate state
//>logSystemState("BUTTON_PRESSED");
//>validateSystemState();
//>}

// ── Global Z speed scale (2.0 = 2x faster => half the delays)
static float Z_SPEED_SCALE = 9.0f;  // set to 1.0 to revert (was 2.0f; currently set to 9)
const int Z_MIN_US = 5;             // don't go below this (driver safe floor, was 10)

//================
// zSpeed function
//================
inline int zSpeed(int baseUs) {
  int scaled = (int)(baseUs / Z_SPEED_SCALE);  // faster -> smaller delays
  if (scaled < Z_MIN_US) scaled = Z_MIN_US;
  return scaled;
}

// ───────────────────────────────────────────────
// Smooth Z-Axis Ramp Movement – Heavy Load Friendly
// ───────────────────────────────────────────────
#define performZMoveWithTimeBasedRamping
#undef performZMoveWithTimeBasedRamping

// ═══════════════ Y-AXIS SPEED RAMP WITH SOLENOID GATING (signature preserved) ═══════════════
// - Signature matches existing call sites: (targetSteps, movePositive, enableSolenoid)
// - V+ scales speed (0.25×..1.25× window via state.velocityScale)
// - Solenoid ON/OFF based on distance traveled (1" buffer on each end)
// - Active-LOW solenoid: LOW=ON, HIGH=OFF
void performYStyleSpeedRamp(long targetSteps, bool movePositive, bool enableSolenoid) {
  long currentPos = position.currentY;
  long stepsToMove = labs(targetSteps - currentPos);
  if (stepsToMove == 0) {
    Serial.println("ℹ️ Y ramp: no movement required");
    return;
  }
  int vplus = constrain(state.velocityScale, 0, 10);
  float yScale = max(0.25f, 1.0f + 0.25f * vplus);
  const float BASE_US = Y_PRECISION_DELAY, MIN_US = Y_FAST_DELAY, MAX_US = 2000.0f;
  uint32_t stepDelayUs = (uint32_t)constrain((long)(BASE_US / yScale), (long)MIN_US, (long)MAX_US);
  digitalWrite(Y_DIR, movePositive ? HIGH : LOW);
  long newHomeY = getNewHomeAbsoluteY();  // Changed from getCompensatedNewHomeY() to direct FRAM coordinates
  long areaWidthSteps = (long)(state.areaWidth * Y_STEPS_PER_INCH);
  long rightEdge = newHomeY, leftEdge = newHomeY - areaWidthSteps;
  long startOff = (bufferZones.isValid && bufferZones.sprayStartOffset > 0) ? bufferZones.sprayStartOffset : (long)(1.0 * Y_STEPS_PER_INCH);
  long endOff = (bufferZones.isValid && bufferZones.sprayEndOffset > 0) ? bufferZones.sprayEndOffset : (long)(1.0 * Y_STEPS_PER_INCH);
  long onPos, offPos;
  if (movePositive) {
    onPos = leftEdge - startOff;
    offPos = rightEdge + endOff;
  } else {
    onPos = rightEdge + startOff;
    offPos = leftEdge - endOff;
  }
  long segMin = min(currentPos, targetSteps), segMax = max(currentPos, targetSteps);
  onPos = constrain(onPos, segMin, segMax);
  offPos = constrain(offPos, segMin, segMax);
  Serial.print("🎯 Y ramp to ");
  Serial.print(targetSteps);
  Serial.print(" (V+ ");
  Serial.print(yScale, 3);
  Serial.print(", stepDelay ");
  Serial.print(stepDelayUs);
  Serial.println("µs)");
  digitalWrite(SOLENOID_PIN, HIGH);  // OFF
  yAxisActive = true;
  bool solenoidActive = false;
  bool solenoidOnTriggered = false;   // Prevent ON spam
  bool solenoidOffTriggered = false;  // Prevent OFF spam

  for (long s = 0; s < stepsToMove; s++) {
    uint32_t currentDelay;
    long accelPhase = stepsToMove / 4, decelPhase = stepsToMove / 4;
    if (s < accelPhase) {
      float p = (float)s / max(1L, accelPhase);
      currentDelay = MAX_STEP_DELAY - (p * (MAX_STEP_DELAY - stepDelayUs));
    } else if (s > (stepsToMove - decelPhase)) {
      float p = (float)(s - (stepsToMove - decelPhase)) / max(1L, decelPhase);
      currentDelay = stepDelayUs + (p * (MAX_STEP_DELAY - stepDelayUs));
    } else {
      currentDelay = stepDelayUs;
    }
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(20);
    digitalWrite(Y_STEP, LOW);
    delayMicroseconds(currentDelay);
    position.currentY += (movePositive ? 1 : -1);
    long yNow = position.currentY;

    // Single-trigger ON logic
    if (enableSolenoid && !solenoidActive && !solenoidOnTriggered) {
      if ((movePositive && yNow >= onPos) || (!movePositive && yNow <= onPos)) {
        digitalWrite(SOLENOID_PIN, LOW);  // ON
        solenoidActive = true;
        solenoidOnTriggered = true;  // Prevent re-triggering
        Serial.print("💦 ON @Y=");
        Serial.println(yNow);
      }
    }

    // Single-trigger OFF logic
    if (enableSolenoid && solenoidActive && !solenoidOffTriggered) {
      if ((movePositive && yNow >= offPos) || (!movePositive && yNow <= offPos)) {
        digitalWrite(SOLENOID_PIN, HIGH);  // OFF
        solenoidActive = false;
        solenoidOffTriggered = true;  // Prevent re-triggering
        Serial.print("💦 OFF @Y=");
        Serial.println(yNow);
      }
    }
  }
  if (solenoidActive) { digitalWrite(SOLENOID_PIN, HIGH); }
  yAxisActive = false;
  toggleTracker.lastRecordedY = position.currentY;
  Serial.println("✅ Y ramp complete");
}

// ═══════════════ UPDATED Z-DROP WITH TRUE S-CURVE SYMMETRY ═══════════════
void performZDropWithSymmetry(float dropInches) {
  Serial.print("🔻 Z-DROP: ");
  Serial.print(dropInches);
  Serial.println("\"");

  long totalSteps = (long)(dropInches * Z_STEPS_PER_INCH + 0.5f);
  if (totalSteps <= 0) {
    Serial.println("⚠️ Drop ≤ 0 — aborting");
    return;
  }

  // Shape fractions (10% up, 10% down)
  const float accelFrac = 0.15f, decelFrac = 0.15f;  // longer accel/decel, shorter cruise at 0.25f

  // Local ramp window (shape target); zSpeed() will clamp ≥ Z_MIN_US globally
  const int vMinDelay = 50, vMaxDelay = 10;  // Was vMinDelay 200

  // Local safety floor for heavy Z-drop under load.
  // Keeps global Z fast (Z_MIN_US stays 10), but we won’t step faster than this here.
  const int Z_DROP_LOCAL_FLOOR_US = 20;  // was 50, now set at 20; if super-solid, 40; if chattery, 60–80

  // Direction = DOWN, and give the driver setup time so we don't miss the first step
  digitalWrite(Z_DIR, LOW);
  delayMicroseconds(20);
  const int zSign = -1;

  // Compute segment sizes with guards for tiny step counts
  long accelSteps = (long)(totalSteps * accelFrac);
  long decelSteps = (long)(totalSteps * decelFrac);
  if (accelSteps < 1) accelSteps = 1;
  if (decelSteps < 1) decelSteps = 1;
  long cruiseSteps = totalSteps - accelSteps - decelSteps;
  if (cruiseSteps < 0) cruiseSteps = 0;

  auto pulse = [&](int delayUs) {
    int d = zSpeed(delayUs);                                   // V+ scaling + global clamp (>= Z_MIN_US)
    if (d < Z_DROP_LOCAL_FLOOR_US) d = Z_DROP_LOCAL_FLOOR_US;  // local floor just for this routine
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(Z_PULSE_WIDTH);
    digitalWrite(Z_STEP, LOW);
    ++ZPulseTotal;  // <— ADD: count one Z pulse
    delayMicroseconds(d);
    position.currentZ += zSign;
  };

  // accel
  for (long i = 0; i < accelSteps; ++i) {
    float phase = (float)i / (float)accelSteps;
    int dUs = (int)(vMinDelay - phase * (vMinDelay - vMaxDelay));
    pulse(dUs);
  }
  // cruise
  for (long i = 0; i < cruiseSteps; ++i) pulse(vMaxDelay);
  // decel
  for (long i = 0; i < decelSteps; ++i) {
    float phase = (float)i / (float)decelSteps;
    int dUs = (int)(vMaxDelay + phase * (vMinDelay - vMaxDelay));
    pulse(dUs);
  }

  Serial.print("✅ Z-DROP done. New Z=");
  Serial.println(position.currentZ);
}

// ═══════════════ SCROLL MESSAGE ═══════════════
void startScrollingMessage() {
  scrollMessage.isActive = true;
  scrollMessage.position = 0;
  scrollMessage.lastUpdate = millis();
}

// ═══════════════ DYNAMIC ENDSTOP DATA ═══════════════
void loadDynamicEndstopData() {
  uint8_t calibrationFlag = fram.read8(FRAM_ENDSTOP_DISTANCES + 20);
  if (calibrationFlag == 1) {
    dynamicEndstops.yAverageDistance = fram.read32(FRAM_ENDSTOP_DISTANCES + 0);
    dynamicEndstops.zAverageDistance = fram.read32(FRAM_ENDSTOP_DISTANCES + 4);
    dynamicEndstops.totalHomingCycles = fram.read32(FRAM_ENDSTOP_DISTANCES + 8);
    dynamicEndstops.consecutiveGoodMeasurements = fram.read8(FRAM_ENDSTOP_DISTANCES + 12);
    dynamicEndstops.isCalibrated = true;

    Serial.print("📏 Loaded dynamic endstop Y=");
    Serial.print((float)dynamicEndstops.yAverageDistance / Y_STEPS_PER_INCH);
    Serial.print("\", Z=");
    Serial.print((float)dynamicEndstops.zAverageDistance / Z_STEPS_PER_INCH);
    Serial.println("\"");
  } else {
    Serial.println("📝 No endstop data; learning on first homing");
    dynamicEndstops.isCalibrated = false;
    for (int i = 0; i < 10; i++) {
      dynamicEndstops.yDistanceHistory[i] = 0;
      dynamicEndstops.zDistanceHistory[i] = 0;
    }
  }
}

#ifndef PERSIST_DYNAMIC_ENDSTOPS
#define PERSIST_DYNAMIC_ENDSTOPS 0
#endif

void saveDynamicEndstopData() {
#if PERSIST_DYNAMIC_ENDSTOPS
  fram.write32(FRAM_ENDSTOP_DISTANCES + 0, dynamicEndstops.yAverageDistance);
  fram.write32(FRAM_ENDSTOP_DISTANCES + 4, dynamicEndstops.zAverageDistance);
  fram.write32(FRAM_ENDSTOP_DISTANCES + 8, dynamicEndstops.totalHomingCycles);
  fram.write8(FRAM_ENDSTOP_DISTANCES + 12, dynamicEndstops.consecutiveGoodMeasurements);
  fram.write8(FRAM_ENDSTOP_DISTANCES + 20, 1);
  Serial.println("💾 Endstop data saved to FRAM");
#else
  Serial.println("ℹ️ Endstop data updated (RAM only)");
#endif
}

// ═══════════════ MISSING IMPLEMENTATIONS TO RESOLVE LINK ERRORS ═══════════════

// If any code calls the int version, route to the float version:
void performZDropWithSymmetry(int dropInches) {
  performZDropWithSymmetry((float)dropInches);
}

// Axis mutual exclusion and switch-delay guard used by setup()/loop()
bool isSafeToMove(bool yRequested, bool zRequested) {
  unsigned long now = millis();

  // If both requested at once, reject (no movement)
  if (yRequested && zRequested) {
    Serial.println("⚠️ MUTUAL EXCLUSION: Both Y and Z requested simultaneously - NO MOVEMENT");
    return false;
  }

  // Enforce the axis switch deadtime only during automated sequences
  if (systemArmed) {
    if (now - lastAxisSwitchTime < AXIS_SWITCH_DELAY) {
      return false;
    }
    if (yRequested || zRequested) {
      lastAxisSwitchTime = now;
    }
  }

  // Track axis lifecycle flags
  if (yRequested) {
    yAxisActive = true;
    zAxisActive = false;
  }
  if (zRequested) {
    zAxisActive = true;
    yAxisActive = false;
  }

  return true;
}

// Dynamic endstop health report used in setup()
void reportEndstopHealth() {
  Serial.println("📊 ═══════════════ ENDSTOP HEALTH REPORT ═══════════════");

  if (dynamicEndstops.isCalibrated) {
    float yInches = (float)dynamicEndstops.yAverageDistance / Y_STEPS_PER_INCH;
    float zInches = (float)dynamicEndstops.zAverageDistance / Z_STEPS_PER_INCH;

    Serial.print("Y avg: ");
    Serial.print(yInches);
    Serial.print("\"   ");
    Serial.print("Z avg: ");
    Serial.print(zInches);
    Serial.println("\"");

    Serial.print("Confidence: ");
    Serial.print(dynamicEndstops.consecutiveGoodMeasurements);
    Serial.println("/10 good samples");

    Serial.print("Total homing cycles: ");
    Serial.println(dynamicEndstops.totalHomingCycles);

    if (dynamicEndstops.significantChangeDetected)
      Serial.println("⚠️ WARNING: Recent distance changes detected");
    if (fabs(dynamicEndstops.yDriftPercent) > 5.0 || fabs(dynamicEndstops.zDriftPercent) > 5.0)
      Serial.println("⚠️ WARNING: Sensor drift >5% detected");
    if (dynamicEndstops.anomalyDetected)
      Serial.println("🚨 CRITICAL: Endstop anomalies detected");

    if (!dynamicEndstops.significantChangeDetected && !dynamicEndstops.anomalyDetected && fabs(dynamicEndstops.yDriftPercent) <= 5.0 && fabs(dynamicEndstops.zDriftPercent) <= 5.0)
      Serial.println("✅ Endstops stable - operating normally");
  } else {
    Serial.println("📝 System not yet calibrated - learning during first homing");
  }

  Serial.println("📊 ═══════════════ REPORT COMPLETE ═══════════════");
}

// (Optional safety) If you declared validateEndstopLogic() but never implemented it, use this:
bool validateEndstopLogic() {
  // Expect pull-ups → HIGH (1) when not triggered
  int zTop = digitalRead(Z_TOP_ENDSTOP);
  int yRight = digitalRead(Y_RIGHT_ENDSTOP);

  Serial.println("🔍 ENDSTOP TEST:");
  Serial.print("Z_TOP: ");
  Serial.println(zTop);
  Serial.print("Y_RIGHT: ");
  Serial.println(yRight);
  Serial.println("(Should all read HIGH when no metal present)");

  // Basic sanity: both should be HIGH at rest
  bool ok = (zTop == HIGH) && (yRight == HIGH);
  if (!ok) Serial.println("⚠️ Endstop sanity check failed - investigate wiring/spacing");
  return ok;
}

// ═══════════════ BUFFER ZONES (SAFETY ONLY) ═══════════════
void establishYAxisBuffers() {
  Serial.println("📏 Establishing Y-Axis safety buffers...");
  long newHomeY = getNewHomeAbsoluteY();

  bufferZones.rightBufferY = newHomeY + (3 * Y_STEPS_PER_INCH);
  bufferZones.leftBufferY = newHomeY - (3 * Y_STEPS_PER_INCH);
  bufferZones.isValid = true;

  Serial.print("📏 Right buffer: ");
  Serial.println(bufferZones.rightBufferY);
  Serial.print("📏 Left buffer: ");
  Serial.println(bufferZones.leftBufferY);
}

#ifndef PERSIST_BUFFER_ZONES
#define PERSIST_BUFFER_ZONES 0  // 0 = RAM only; 1 = actually load from FRAM
#endif

// Safe loader so setup() link succeeds whether persistence is on or off
void loadBufferZonesFromFRAM() {
#if PERSIST_BUFFER_ZONES
  // Expect layout: [0]rightY(int32) [4]leftY(int32) [8]startOff(int32) [12]endOff(int32) [16]flag(uint8)
  if (fram.read8(FRAM_BUFFER_ZONES + 16) == 1) {
    bufferZones.rightBufferY = fram.read32(FRAM_BUFFER_ZONES + 0);
    bufferZones.leftBufferY = fram.read32(FRAM_BUFFER_ZONES + 4);
    bufferZones.sprayStartOffset = fram.read32(FRAM_BUFFER_ZONES + 8);
    bufferZones.sprayEndOffset = fram.read32(FRAM_BUFFER_ZONES + 12);
    bufferZones.isValid = true;
    Serial.println("✅ Buffer zones loaded from FRAM");
  } else {
    bufferZones.isValid = false;
    Serial.println("ℹ️ No buffer zones in FRAM; using RAM defaults");
  }
#else
  bufferZones.isValid = false;  // RAM-only mode: compute from NEW HOME when needed
  Serial.println("ℹ️ Buffer zone persistence disabled (RAM only)");
#endif
}

void establishRunBuffers(long startY, int areaWidth) {
  Serial.println("🛡️ Calculating fresh buffer zones for this run...");

  // Use absolute NEW HOME coordinates, not current position
  long newHomeY = getNewHomeAbsoluteY();
  long areaWidthSteps = areaWidth * Y_STEPS_PER_INCH;
  long rightEdge = newHomeY;                  // Always from NEW HOME
  long leftEdge = newHomeY - areaWidthSteps;  // Always relative to NEW HOME



  // CORRECTED: 3" ramp zone + 1" solenoid zone = 4" total buffer
  bufferZones.rightBufferY = rightEdge + (4 * Y_STEPS_PER_INCH);  // 3" ramp + 1" solenoid
  bufferZones.leftBufferY = leftEdge - (4 * Y_STEPS_PER_INCH);    // 3" ramp + 1" solenoid

  // 1" spray activation offsets (solenoid ON/OFF zones)
  bufferZones.sprayStartOffset = 1 * Y_STEPS_PER_INCH;  // 1" before substrate
  bufferZones.sprayEndOffset = 1 * Y_STEPS_PER_INCH;    // 1" after substrate
  bufferZones.isValid = true;

  Serial.print("🛡️ Right buffer: ");
  Serial.print(bufferZones.rightBufferY);
  Serial.print(", Left buffer: ");
  Serial.println(bufferZones.leftBufferY);
}

void validateBufferCoordinates() {
  if (!bufferZones.isValid) return;

  // Check for reasonable coordinates (within machine limits)
  long maxY = TOTAL_Y_TRAVEL_INCHES * Y_STEPS_PER_INCH;
  long minY = -maxY;  // Allow negative coordinates for NEW HOME positioning

  if (bufferZones.rightBufferY > maxY || bufferZones.rightBufferY < minY || bufferZones.leftBufferY > maxY || bufferZones.leftBufferY < minY) {

    Serial.println("⚠️ WARNING: Buffer coordinates outside machine limits!");
    Serial.print("   Right buffer: ");
    Serial.println(bufferZones.rightBufferY);
    Serial.print("   Left buffer: ");
    Serial.println(bufferZones.leftBufferY);
    Serial.print("   Machine limits: ±");
    Serial.println(maxY);

    bufferZones.isValid = false;
  }
}

// ═══════════════ SIMPLE REPEATER FUNCTIONS ═══════════════
void simpleSavePosition() {
  Serial.println("SIMPLE SAVE: Recording current position AND parameters");

  long offsetY = position.currentY - position.trueHomeY;
  long offsetZ = position.currentZ - position.trueHomeZ;

  Serial.print("Saving: Y=");
  Serial.print(offsetY);
  Serial.print(" Z=");
  Serial.println(offsetZ);

  // Save coordinates
  fram.write32(FRAM_NEW_HOME_Y_OFFSET, offsetY);
  fram.write32(FRAM_NEW_HOME_Z_OFFSET, offsetZ);

  // Save current parameters too
  fram.write8(FRAM_PROGRAM_DATA + 0, state.sprayHeightInches);
  fram.write8(FRAM_PROGRAM_DATA + 1, state.coveragePercent);
  fram.write8(FRAM_PROGRAM_DATA + 2, state.areaWidth);
  fram.write8(FRAM_PROGRAM_DATA + 3, state.areaHeight);
  fram.write8(FRAM_PROGRAM_DATA + 4, state.velocityScale);
  fram.write8(FRAM_PROGRAM_DATA + 5, state.repeatCount);

  fram.write8(FRAM_VALID_FLAG, 1);

  position.newHomeOffsetY = offsetY;
  position.newHomeOffsetZ = offsetZ;

  Serial.print("Saved params: SH=");
  Serial.print(state.sprayHeightInches);
  Serial.print(" CP=");
  Serial.print(state.coveragePercent);
  Serial.print(" AW=");
  Serial.print(state.areaWidth);
  Serial.print(" AH=");
  Serial.println(state.areaHeight);
}

void simpleGoToSavedPosition() {
  Serial.println("SIMPLE GO: Moving to saved position");

  long targetY = position.trueHomeY + position.newHomeOffsetY;
  long targetZ = position.trueHomeZ + position.newHomeOffsetZ;

  Serial.print("Going to: Y=");
  Serial.print(targetY);
  Serial.print(" Z=");
  Serial.println(targetZ);

  // Move Z first
  long zStepsNeeded = targetZ - position.currentZ;
  if (abs(zStepsNeeded) > 10) {
    performZMoveWithTimeBasedRamping(abs(zStepsNeeded), zStepsNeeded > 0);
  }

  // Move Y second
  long yStepsNeeded = targetY - position.currentY;
  if (abs(yStepsNeeded) > 10) {
    digitalWrite(Y_DIR, yStepsNeeded > 0 ? HIGH : LOW);
    for (long i = 0; i < abs(yStepsNeeded); i++) {
      digitalWrite(Y_STEP, HIGH);
      delayMicroseconds(600);
      digitalWrite(Y_STEP, LOW);
      delayMicroseconds(600);
      position.currentY += (yStepsNeeded > 0) ? 1 : -1;
    }
  }

  Serial.println("Arrived at saved position");
}



// ═══════════════ AUTOMATED SEQUENCE CONDUCTOR - THE COMPLETE DANCE ═══════════════
void executeAutomatedSequence() {
  Serial.println("🎭 BEGINNING COMPLETE AUTOMATED SEQUENCE WITH SH LOGIC...");

  // Use live state.* so TRUE-HOME programming applies immediately to pass count
  int calculatedPasses = calculateFinalPassCount(
    state.areaHeight,
    state.sprayHeightInches,
    state.coveragePercent);

  // Check if at TRUE HOME
  float yInchesFromTrueHome = abs(position.currentY - position.trueHomeY) / Y_STEPS_PER_INCH;
  float zInchesFromTrueHome = abs(position.currentZ - position.trueHomeZ) / Z_STEPS_PER_INCH;
  bool nearTrueHome = (yInchesFromTrueHome <= 3.0) && (zInchesFromTrueHome <= 3.0);

  // Allow execution if at TRUE HOME (will recalculate) OR if FRAM is valid (will use stored)
  if (!nearTrueHome && fram.read8(FRAM_VALID_FLAG) != 1) {
    Serial.println("🛑 START blocked: Not at TRUE HOME and no valid program stored");
    return;
  }

  // If at TRUE HOME, recalculate NEW HOME from current parameters
  //>if (nearTrueHome) {
  //>Serial.println("📍 At TRUE HOME - recalculating NEW HOME from current parameters");
  //>calculateNewHomeFromTrueHome();
  //>}

  int currentRepeatCycle = state.repeatCount;
  bool lastDirectionPositive = true;

  Serial.print("🎯 Calculated passes based on overlap: ");
  Serial.print(calculatedPasses);
  Serial.print(" (AH=");
  Serial.print(state.areaHeight);
  Serial.print("in, SH=");
  Serial.print(state.sprayHeightInches);
  Serial.println("in)");

  // Apply V+ speed scaling for this run
  Z_SPEED_SCALE = max(0.25f, 1.0f + 0.25f * state.velocityScale);

  while (currentRepeatCycle > 0) {
    Serial.print("🔄 REPEAT CYCLE ");
    Serial.println(state.repeatCount - currentRepeatCycle + 1);

    // Reset for each repeat cycle to ensure initial drop happens
    bool isFirstPass = true;

    // Will hold the run's top edge Z (absolute steps)
    long runTopEdgeZSteps = 0;

    // ═══════════════ Initial pass positioning (use one pitch) ═══════════════
    if (isFirstPass) {
      float SH = programMemory.sprayHeightInches;
      float CP = state.coveragePercent * 0.01f;
      float pitch = SH * (1.0f - CP);

      Serial.print("🎯 Initial drop (pitch) = ");
      Serial.print(pitch, 3);
      Serial.println(" in");

      performZDropWithSymmetry(pitch);
      delay(AXIS_SWITCH_DELAY);

      // Top edge after first drop = current Z + SH/2
      const long HALF_SH_STEPS = (long)(SH * 0.5f * Z_STEPS_PER_INCH + 0.5f);
      runTopEdgeZSteps = position.currentZ + HALF_SH_STEPS;
      Serial.print("📍 Captured runTopEdgeZSteps = ");
      Serial.println(runTopEdgeZSteps);

      isFirstPass = false;
    }

    // Buffer zones will be established physically during Pass 1
    long bufferZone1 = 0;  // Will be set by Pass 1
    long bufferZone2 = 0;  // Will be set by Pass 1
    Serial.println("Buffers will be established physically during Pass 1");

    // Helper: compute pitch in inches from CP
    auto pitchInches = [&]() {
      float overlap = programMemory.coveragePercent / 100.0f;
      return programMemory.sprayHeightInches * (1.0f - overlap);
    };

    // ───────────────────────────────── PASSES ─────────────────────────────────
    for (int pass = 1; pass <= calculatedPasses; ++pass) {
      Serial.print("Pass ");
      Serial.print(pass);
      Serial.print(" of ");
      Serial.println(calculatedPasses);

      if (pass == 1) {
        Serial.println("Pass 1: RAMP BACK + SPRAY SEQUENCE");

        // Phase 1: Ramp back 2" (no spray)
        long rampBackPos = position.currentY + (2 * Y_STEPS_PER_INCH);
        performYStyleSpeedRamp(rampBackPos, true, false);

        // Phase 2: One continuous movement - ramp forward 2" + substrate + ramp down 2"
        long totalDistance = (2 + state.areaWidth + 2) * Y_STEPS_PER_INCH;
        long sprayEndPos = position.currentY - totalDistance;
        performYStyleSpeedRamp(sprayEndPos, false, true);  // Solenoid ON for middle 80%

        // Set buffers for remaining passes
        bufferZone1 = rampBackPos;
        bufferZone2 = sprayEndPos;

        lastDirectionPositive = false;

      } else {
        // ALL OTHER PASSES: Use established buffers
        if (lastDirectionPositive) {
          Serial.println("Pass: RIGHT → LEFT");
          performYStyleSpeedRamp(bufferZone2, false, true);
          lastDirectionPositive = false;
        } else {
          Serial.println("Pass: LEFT → RIGHT");
          performYStyleSpeedRamp(bufferZone1, true, true);
          lastDirectionPositive = true;
        }
      }

      // Drop for next pass (but not after the final pass)
      if (pass < calculatedPasses) {
        Serial.println("⏸️ 0.5s pause between passes...");
        delay(500);

        float pitch_in = pitchInches();
        delay(AXIS_SWITCH_DELAY);
        Serial.print("🎯 PASS Z-DROP (pitch) = ");
        Serial.print(pitch_in, 3);
        Serial.println("\"");
        performZDropWithSymmetry(pitch_in);
        delay(AXIS_SWITCH_DELAY);
      }
    }  // end for (pass)

    // ─────────── One repeat done (only after ALL passes finished) ───────────
    currentRepeatCycle--;

    if (currentRepeatCycle > 0) {
      Serial.println("🔄 Repeat cycle complete - returning to NEW HOME position");

      // Return Z to NEW HOME (smooth ramp)
      long targetZ = getNewHomeAbsoluteZ();
      long zStepsNeeded = targetZ - position.currentZ;
      if (labs(zStepsNeeded) > 10) {
        Serial.print("🎯 Z return to NEW HOME Z: ");
        Serial.print(labs(zStepsNeeded));
        Serial.println(" steps");
        zAxisActive = true;
        performZMoveWithTimeBasedRamping(labs(zStepsNeeded), (zStepsNeeded > 0));
        zAxisActive = false;
        position.currentZ = targetZ;
        delay(500);
      } else {
        Serial.println("ℹ️ Z return: no movement required");
      }

      // FORCE Y to NEW HOME - no spray, no buffers, direct movement
      long newHomeY = getNewHomeAbsoluteY();
      long currentY = position.currentY;
      long yStepsNeeded = newHomeY - currentY;

      Serial.print("🎯 FORCING Y to NEW HOME: ");
      Serial.print(newHomeY);
      Serial.print(" (currently at ");
      Serial.print(currentY);
      Serial.print(", need ");
      Serial.print(yStepsNeeded);
      Serial.println(" steps)");

      if (labs(yStepsNeeded) > 5) {  // Lower threshold
        digitalWrite(Y_DIR, yStepsNeeded > 0 ? HIGH : LOW);
        yAxisActive = true;

        for (long i = 0; i < labs(yStepsNeeded); i++) {
          digitalWrite(Y_STEP, HIGH);
          delayMicroseconds(20);
          digitalWrite(Y_STEP, LOW);
          delayMicroseconds(800);  // Steady speed
          position.currentY += (yStepsNeeded > 0) ? 1 : -1;
        }

        yAxisActive = false;
        toggleTracker.lastRecordedY = position.currentY;
        Serial.print("✅ Y FORCED to NEW HOME: ");
        Serial.println(position.currentY);
      } else {
        Serial.println("✅ Y already at NEW HOME");
      }
    }
  }  // end while

  // Only go to TRUE HOME when ALL cycles are done
  Serial.println("🎭 ALL REPEAT CYCLES COMPLETE!");
  Serial.println("🏠 Returning to TRUE HOME using boot sequence");
  delay(AXIS_SWITCH_DELAY);

  performTrueHomeSequence();  // Let this handle everything

  currentMode = MANUAL_JOG;
  systemArmed = false;
  startState = false;
  movedToNewHomeThisCycle = false;
  updateAllLEDs();
  // Log button interaction and validate state
  logSystemState("BUTTON_PRESSED");
  validateSystemState();
  startScrollingMessage();

  Serial.println("✅ SEQUENCE COMPLETE - System ready for next operation");
}

// ═══════════════ BOOT SEQUENCE & TEST UTILITIES ═══════════════
void testFRAM() {
  Serial.println("🧪 Testing FRAM...");
  fram.write8(0x00F0, 0xAB);  // safe scratch
  uint8_t readBack = fram.read8(0x00F0);
  Serial.print("Wrote 0xAB, read back 0x");
  Serial.println(readBack, HEX);
  Serial.println(readBack == 0xAB ? "✅ FRAM is working!" : "❌ FRAM test failed!");
}

// ═══════════════ BOOT MARQUEE (3 displays) ═══════════════
void scrollBootMessage() {
  const char* msg = "JOB SECURITY ONLINE, WELCOME TERESSA ";
  int len = strlen(msg);
  for (int pos = -12; pos <= len; ++pos) {
    display1.clear();
    display2.clear();
    display3.clear();
    for (int i = 0; i < 12; ++i) {
      int idx = pos + i;
      char c = (idx >= 0 && idx < len) ? msg[idx] : ' ';
      if (i < 4) display1.writeDigitAscii(i, c);
      else if (i < 8) display2.writeDigitAscii(i - 4, c);
      else display3.writeDigitAscii(i - 8, c);
    }
    display1.writeDisplay();
    display2.writeDisplay();
    display3.writeDisplay();
    delay(100);
  }
  display1.clear();
  display2.clear();
  display3.clear();
  display1.writeDisplay();
  display2.writeDisplay();
  display3.writeDisplay();
}

// ═══════════════ CRITICAL DISPLAY-FAIL INDICATOR ═══════════════
void showErrorLEDs() {
  uint32_t red = programRing.Color(255, 0, 0);
  programRing.setPixelColor(0, red);
  lockinRing.setPixelColor(0, red);
  startRing.setPixelColor(0, red);
  speedRing.setPixelColor(0, red);
  programRing.show();
  lockinRing.show();
  startRing.show();
  speedRing.show();

  Serial.println("ERROR: Display not found!");
  while (true) {
    delay(500);
    programRing.setBrightness(0);
    lockinRing.setBrightness(0);
    startRing.setBrightness(0);
    speedRing.setBrightness(0);
    programRing.show();
    lockinRing.show();
    startRing.show();
    speedRing.show();
    delay(500);
    programRing.setBrightness(BRIGHTNESS);
    lockinRing.setBrightness(BRIGHTNESS);
    startRing.setBrightness(BRIGHTNESS);
    speedRing.setBrightness(BRIGHTNESS);
    programRing.show();
    lockinRing.show();
    startRing.show();
    speedRing.show();
  }
}

// ═══════════════ MASTER TRUE-HOME SEQUENCE ═══════════════
void performTrueHomeSequence() {
  Serial.println("🏠 TRUE-HOME (ramped)…");
  //>allowZSensingDuringMoves = true;  // enable sensing reactions during homing

  /* Speed constants */
  const int Y_FLAT_US = 800;   // trusted belt speed
  const int Y_SLOW_US = 1200;  // Y brake & back-off
  const int Z_FLAT_US = 10;    // baseline Z interstep delay (tune)
  const int Z_SLOW_US = 30;    // slow zone when sensing is active (tune)

  // ✅ SHARED VARIABLES - Declare ALL at top of function
  //>bool zSensedOnce = false, ySensedOnce = false;
  //>uint8_t zCnt = 0, yCnt = 0;
  //>bool zCand = HIGH, yCand = HIGH;
  //>const uint8_t SENSE_CONSEC_REQUIRED = 3;  // ~15ms at ~5ms sample
  //>bool zSensingTriggered = false;
  //>bool ySensingTriggered = false;

  // ✅ Z RAMPING VARIABLES
  static unsigned long rampStartTime = 0;
  static bool rampInitialized = false;

  /* ══════════════════════════════════════════════════════════════ */
  /* ── Z-AXIS: Ramped movement to TRUE HOME ───────────────────── */
  /* ══════════════════════════════════════════════════════════════ */
  digitalWrite(Z_DIR, LOW);  // DOWN to TRUE HOME

  while (digitalRead(Z_TOP_ENDSTOP) == HIGH) {
    int dUs;  // Z delay value

    // ✅ RAMPING LOGIC
    if (!rampInitialized) {
      rampStartTime = micros();
      rampInitialized = true;
    }

    unsigned long elapsed = micros() - rampStartTime;
    if (elapsed < 1000000UL) {  // 1 second ramp
      float progress = (float)elapsed / 1000000.0f;
      dUs = 50 - (progress * 40);  // 50µs → 10µs over 1 second
    } else {
      dUs = Z_FLAT_US;  // Cruise speed
    }

    // ✅ Z SENSING LOGIC
    //>if (allowZSensingDuringMoves) {
    //>bool zRaw = digitalRead(Z_SENSING_ENDSTOP);
    //>if (zRaw == zCand) {
    //>if (zCnt < 255) zCnt++;
    //>} else {
    //>zCand = zRaw;
    //>zCnt = 1;
    //>}
    //>if (!zSensedOnce && zCnt >= SENSE_CONSEC_REQUIRED && zCand == LOW) {
    //>zSensedOnce = true;
    //>zSensingTriggered = true;
    //>Serial.println("🎯 Z sensing triggered - slowing to crawl speed");
    //>}
    //>}

    // ✅ SENSING OVERRIDE (overrides ramping)
    //>if (zSensingTriggered) {
    //>dUs = Z_SLOW_US;
    //>}

    // Z pulse
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(Z_PULSE_WIDTH);
    digitalWrite(Z_STEP, LOW);
    delayMicroseconds(dUs);
    --position.currentZ;
  }

  //>if (!zSensedOnce) {
  //>Serial.println("⚠️ Z_SENSING never triggered – check alignment");
  //>}

  /* ✅ Z BACK-OFF 0.40″ */
  digitalWrite(Z_DIR, HIGH);  // UP (away from lower endstop)
  delayMicroseconds(20);      // DIR setup
  for (long i = 0; i < Z_BACKOFF_STEPS; ++i) {
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(Z_PULSE_WIDTH);
    digitalWrite(Z_STEP, LOW);
    delayMicroseconds(Z_SLOW_US);  // fixed, not V+-scaled (good)
    ++position.currentZ;
  }
  position.currentZ = 0;
  rampInitialized = false;

  delay(AXIS_SWITCH_DELAY);  // Brief pause between axes

  /* ══════════════════════════════════════════════════════════════ */
  /* ── Y-AXIS: Flat movement to TRUE HOME ──────────────────────── */
  /* ══════════════════════════════════════════════════════════════ */
  digitalWrite(Y_DIR, HIGH);  // RIGHT

  while (digitalRead(Y_RIGHT_ENDSTOP) == HIGH) {
    int dUs = Y_FLAT_US;  // Y delay value (flat speed, no ramping)

    // ✅ Y SENSING LOGIC (same structure as Z)
    //>bool yRaw = digitalRead(Y_SENSING_ENDSTOP);
    //>if (yRaw == yCand) {
    //>if (yCnt < 255) yCnt++;
    //>} else {
    //>yCand = yRaw;
    //>yCnt = 1;
    //>}
    //>if (!ySensedOnce && yCnt >= SENSE_CONSEC_REQUIRED && yCand == LOW) {
    //>ySensedOnce = true;
    //>ySensingTriggered = true;
    //>Serial.println("🎯 Y sensing triggered - slowing to crawl speed");
    //>}

    // ✅ Y SENSING OVERRIDE
    //>if (ySensingTriggered) {
    //>dUs = Y_SLOW_US;
    //>}

    // Y pulse
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(Y_PULSE_WIDTH);
    digitalWrite(Y_STEP, LOW);
    delayMicroseconds(dUs);

    ++position.currentY;
  }

  //>if (!ySensedOnce) {
  //>Serial.println("⚠️ Y_SENSING never triggered – check alignment");
  //>}

  /* ✅ Y BACK-OFF 0.40″ */
  digitalWrite(Y_DIR, LOW);  // LEFT
  for (long i = 0; i < long(0.40f * Y_STEPS_PER_INCH + 0.5f); ++i) {
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(Y_PULSE_WIDTH);
    digitalWrite(Y_STEP, LOW);
    delayMicroseconds(Y_SLOW_US);
    --position.currentY;
  }
  position.currentY = 0;
  position.currentZ = 0;  // Also ensure Z is set

  // 🔧 FIX: Update toggle tracker when TRUE HOME is established
  toggleTracker.lastRecordedY = 0;
  toggleTracker.lastRecordedZ = 0;
  toggleTracker.hasMovement = true;
  // Log TRUE HOME establishment to FRAM
  logPositionToFRAM(POS_OP_TRUE_HOME);
  logSystemState("TRUE_HOME_ESTABLISHED");
  Serial.println("TRUE HOME position logged to FRAM memory palace");
  Serial.println("🔧 POSITION DRIFT FIX: Toggle tracker reset to TRUE HOME (0,0)");

  // ✅ CRITICAL: Reload NEW HOME offset after TRUE HOME is established
  position.newHomeOffsetY = fram.read32(FRAM_NEW_HOME_Y_OFFSET);
  position.newHomeOffsetZ = fram.read32(FRAM_NEW_HOME_Z_OFFSET);

  Serial.print("📥 LOADED NEW HOME OFFSET: Y=");
  Serial.print(position.newHomeOffsetY);
  Serial.print(", Z=");
  Serial.println(position.newHomeOffsetZ);

  /* ✅ COMPLETION */
  saveTrueHomeToFRAM();
  Serial.println("✅ TRUE-HOME complete → (0,0)");
  //>allowZSensingDuringMoves = false;

  // ✅ PRESERVE SYSTEM STATE - Don't reset after dance
  Serial.println("🔒 System state preserved - dance complete");

  // ✅ ADD THESE LINES:
  lockState = false;         // Reset lock when at TRUE HOME
  systemLocked = false;      // Unlock system for normal operation
  currentMode = MANUAL_JOG;  // Return to manual jog mode
  updateAllLEDs();           // Update LED display to show unlocked state
  // Log button interaction and validate state
  logSystemState("BUTTON_PRESSED");
  validateSystemState();
  Serial.println("🔓 LOCK STATE RESET - System ready for new programming");
}

// 🔍 Position Consistency Monitor
void monitorPositionConsistency() {
  static unsigned long lastReport = 0;

  // Check for position drift every 3 seconds
  if (millis() - lastReport > 3000) {
    long yDrift = abs(position.currentY - toggleTracker.lastRecordedY);
    long zDrift = abs(position.currentZ - toggleTracker.lastRecordedZ);

    if (yDrift > 5 || zDrift > 5) {
      Serial.println("⚠️ POSITION DRIFT DETECTED!");
      Serial.print("   Current Y=");
      Serial.print(position.currentY);
      Serial.print(", Tracker Y=");
      Serial.println(toggleTracker.lastRecordedY);
      Serial.print("   Current Z=");
      Serial.print(position.currentZ);
      Serial.print(", Tracker Z=");
      Serial.println(toggleTracker.lastRecordedZ);
      Serial.print("   Drift: Y=");
      Serial.print(yDrift);
      Serial.print(", Z=");
      Serial.println(zDrift);
    }

    lastReport = millis();
  }
}

// 🔍 Endstop State Monitor
//>void monitorEndstopStates() {
//>static bool lastYSensing = HIGH, lastZSensing = HIGH;
//>static bool lastYTrue = HIGH, lastZTrue = HIGH;
//>static unsigned long lastEndstopReport = 0;

//>bool currentYSensing = digitalRead(Y_SENSING_ENDSTOP);
//>bool currentZSensing = digitalRead(Z_SENSING_ENDSTOP);
//>bool currentYTrue = digitalRead(Y_RIGHT_ENDSTOP);
//>bool currentZTrue = digitalRead(Z_TOP_ENDSTOP);

// Log any endstop changes
//>if (currentYSensing != lastYSensing) {
//>Serial.print("🎯 Y_SENSING: ");
//>Serial.println(currentYSensing ? "HIGH" : "LOW");
//>lastYSensing = currentYSensing;
//>}

//>if (currentZSensing != lastZSensing) {
//>Serial.print("🎯 Z_SENSING: ");
//>Serial.println(currentZSensing ? "HIGH" : "LOW");
//>lastZSensing = currentZSensing;
//>}

//>if (currentYTrue != lastYTrue) {
//>Serial.print("🏠 Y_TRUE_HOME: ");
//>Serial.println(currentYTrue ? "HIGH" : "LOW");
//>lastYTrue = currentYTrue;
//>}

//>if (currentZTrue != lastZTrue) {
//>Serial.print("🏠 Z_TRUE_HOME: ");
//>Serial.println(currentZTrue ? "HIGH" : "LOW");
//>lastZTrue = currentZTrue;
//>}

// Periodic status report
//>if (millis() - lastEndstopReport > 10000) { // Every 10 seconds
//>if (currentDebugLevel >= DEBUG_VERBOSE) {
//>Serial.println("📊 ENDSTOP STATUS:");
//>Serial.print("   Y_SENSING="); Serial.print(currentYSensing ? "HIGH" : "LOW");
//>Serial.print(", Z_SENSING="); Serial.print(currentZSensing ? "HIGH" : "LOW");
//>Serial.print(", Y_TRUE="); Serial.print(currentYTrue ? "HIGH" : "LOW");
//>Serial.print(", Z_TRUE="); Serial.println(currentZTrue ? "HIGH" : "LOW");
//>}
//>lastEndstopReport = millis();
//>}
//>}

void setup() {
  Serial.begin(115200);
  // ─────────── UART SANITY ───────────
  // Only Serial (USB) is active. Serial1 is intentionally unused so pins 0/1 stay as GPIO.
  // This ensures SOLENOID_PIN (0) is safe and won’t be hijacked by UART1.

  fram.begin();
  testFRAM();

  // FRAM will now remember user inputs - no auto-clearing
  // Serial.println("🔧 Clearing potentially corrupted NEW HOME data...");
  // fram.write8(FRAM_VALID_FLAG, 0);  // DISABLED: Let FRAM remember user settings
  //fram.write32(FRAM_NEW_HOME_Y_OFFSET, 0);
  //fram.write32(FRAM_NEW_HOME_Z_OFFSET, 0);
  Serial.println("✅ Preserving saved NEW HOME data for continuation");

  // ADD THE TEST PARAMETERS RIGHT HERE INSIDE SETUP():
  //Serial.println("🔧 Setting test parameters for verification...");
  //state.sprayHeightInches = 3;
  //state.coveragePercent = 50;
  //state.areaWidth = 24;
  //state.areaHeight = 18;
  //state.velocityScale = 0;
  //state.repeatCount = 1;

  // Save test parameters
  //fram.write8(FRAM_PROGRAM_DATA + 0, state.sprayHeightInches);
  //fram.write8(FRAM_PROGRAM_DATA + 1, state.coveragePercent);
  //fram.write8(FRAM_PROGRAM_DATA + 2, state.areaWidth);
  //fram.write8(FRAM_PROGRAM_DATA + 3, state.areaHeight);
  //fram.write8(FRAM_PROGRAM_DATA + 4, state.velocityScale);
  //fram.write8(FRAM_PROGRAM_DATA + 5, state.repeatCount);
  //fram.write8(FRAM_VALID_FLAG, 1);  // Mark as valid
  //Serial.println("✅ Test parameters saved to FRAM");

  //═══════════════ Load saved data ═══════════════
  //Disabled automatic FRAM offset loading at boot – will apply only after homing
  loadTrueHomeFromFRAM();
  loadNewHomeOffsetFromFRAM();
  featureFlags.loadFromFRAM();

  // Initialize FRAM memory palace systems
  Serial.println("Initializing FRAM Memory Palace...");
  logSystemState("SYSTEM_BOOT");
  Serial.println("FRAM Memory Palace online - smart position tracking active");
  Serial.println("Drift detection: ACTIVE (10-step threshold)");
  Serial.println("State validation: ACTIVE (conflict prevention)");
  Serial.println("Memory capacity: 35KB allocated from 512KB total");

  // ═══════════════ Initialize Wire2 for I2C communication ═══════════════
  Wire2.setClock(50000);
  Wire2.begin();
  delay(100);

  // Initialize FRAM memory palace systems
  Serial.println("Initializing FRAM Memory Palace...");
  logSystemState("SYSTEM_BOOT");
  Serial.println("FRAM Memory Palace online - smart position tracking active");
  Serial.println("Drift detection: ACTIVE (10-step threshold)");
  Serial.println("State validation: ACTIVE (conflict prevention)");
  Serial.println("Memory capacity: 35KB allocated from 512KB total");
  Serial.println("🔒 FRAM offsets will be loaded after successful TRUE HOME");

  // Test endstop readings during startup
  Serial.println("🔍 ENDSTOP TEST:");
  //>Serial.print("Y_SENSING: ");
  //>Serial.println(digitalRead(Y_SENSING_ENDSTOP));
  Serial.print("Z_TOP: ");
  Serial.println(digitalRead(Z_TOP_ENDSTOP));
  //>Serial.print("Z_SENSING: ");
  //>Serial.println(digitalRead(Z_SENSING_ENDSTOP));
  Serial.print("Y_RIGHT: ");
  Serial.println(digitalRead(Y_RIGHT_ENDSTOP));
  Serial.println("(Should all read HIGH when no metal present)");

  // pinMode(POWER_MONITOR_PIN, INPUT); // DISABLED: No physical circuit
  // checkForEmergencyRecovery(); // DISABLED: Related to phantom power monitoring

  // (existing boot prints)
  Serial.println("⚙️ Z CONFIG:");
  Serial.print("  MICROSTEPS/REV = ");
  Serial.println((int)Z_MICROSTEPS_PER_REV);
  Serial.print("  GEAR RATIO     = ");
  Serial.println(Z_GEAR_RATIO, 1);
  Serial.print("  PINION in/rev  = ");
  Serial.println(Z_PINION_TRAVEL_PER_REV_IN, 3);
  Serial.print("  Z_STEPS/IN     = ");
  Serial.println((long)Z_STEPS_PER_INCH);
  Serial.print("  Z_BACKOFF_IN   = ");
  Serial.println(Z_BACKOFF_INCH, 3);
  Serial.print("  Z_BACKOFF_STEPS= ");
  Serial.println(Z_BACKOFF_STEPS);

  // ═══════════════ Initialize NeoPixels ═══════════════
  programRing.begin();
  lockinRing.begin();
  startRing.begin();
  speedRing.begin();
  programRing.setBrightness(BRIGHTNESS);
  lockinRing.setBrightness(BRIGHTNESS);
  startRing.setBrightness(BRIGHTNESS);
  speedRing.setBrightness(BRIGHTNESS);
  programRing.show();
  lockinRing.show();
  startRing.show();
  speedRing.show();

  // ═══════════════ Initialize ALL THREE displays with debug output ═══════════════
  Serial.println("🖥️ Initializing Display 1 (0x70)...");
  if (!display1.begin(0x70, &Wire2)) {
    Serial.println("❌ Display 1 failed to initialize!");
    showErrorLEDs();
  } else {
    Serial.println("✅ Display 1 initialized successfully");
  }

  Serial.println("🖥️ Initializing Display 2 (0x72)...");
  if (!display2.begin(0x72, &Wire2)) {
    Serial.println("❌ Display 2 failed to initialize!");
    showErrorLEDs();
  } else {
    Serial.println("✅ Display 2 initialized successfully");
  }

  Serial.println("🖥️ Initializing Display 3 (0x74)...");
  if (!display3.begin(0x74, &Wire2)) {
    Serial.println("❌ Display 3 failed to initialize!");
    showErrorLEDs();
  } else {
    Serial.println("✅ Display 3 initialized successfully");
  }

  display1.setBrightness(5);
  display2.setBrightness(5);
  display3.setBrightness(5);
  display1.clear();
  display2.clear();
  display3.clear();
  display1.writeDisplay();
  display2.writeDisplay();
  display3.writeDisplay();

  // ═══════════════ Initialize all input pins ═══════════════
  pinMode(PROGRAM_BTN, INPUT_PULLUP);
  pinMode(LOCKIN_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(SPEED_BTN, INPUT_PULLUP);

  programDebouncer.attach(PROGRAM_BTN);
  programDebouncer.interval(25);
  lockinDebouncer.attach(LOCKIN_BTN);
  lockinDebouncer.interval(25);
  startDebouncer.attach(START_BTN);
  startDebouncer.interval(25);
  speedDebouncer.attach(SPEED_BTN);
  speedDebouncer.interval(25);

  // ═══════════════ Motor and jog pins ═══════════════
  pinMode(Y_LEFT, INPUT_PULLUP);
  pinMode(Y_RIGHT, INPUT_PULLUP);
  pinMode(Z_DOWN, INPUT_PULLUP);
  pinMode(Z_UP, INPUT_PULLUP);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);

  // ═══════════════ Initialize solenoid pin ═══════════════
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, HIGH);  // ═══════════════ Initialize OFF (active LOW relay) - Ensure solenoid starts OFF ═══════════════

  // ═══════════════ Initialize endstop pins ═══════════════
  //>pinMode(Y_SENSING_ENDSTOP, INPUT_PULLUP);  // Y-axis sensing endstop
  pinMode(Z_TOP_ENDSTOP, INPUT_PULLUP);  // TRUE HOME Z
  //>pinMode(Z_SENSING_ENDSTOP, INPUT_PULLUP);  // Z-axis sensing endstop
  pinMode(Y_RIGHT_ENDSTOP, INPUT_PULLUP);  // TRUE HOME Y

  Serial.println("🔧 Endstop pins initialized with pullup resistors");

  // ═══════════════ Initialize intelligent endstop system ═══════════════
  Serial.println("🧠 Initializing intelligent endstop system...");
  loadDynamicEndstopData();
  reportEndstopHealth();

  // ═══════════════ Read initial encoder states ═══════════════
  pinMode(ENC1_CLK, INPUT_PULLUP);
  pinMode(ENC1_DT, INPUT_PULLUP);
  pinMode(ENC1_SW, INPUT_PULLUP);
  pinMode(ENC2_CLK, INPUT_PULLUP);
  pinMode(ENC2_DT, INPUT_PULLUP);
  pinMode(ENC2_SW, INPUT_PULLUP);
  pinMode(ENC3_CLK, INPUT_PULLUP);
  pinMode(ENC3_DT, INPUT_PULLUP);
  pinMode(ENC3_SW, INPUT_PULLUP);
  pinMode(ENC4_CLK, INPUT_PULLUP);
  pinMode(ENC4_DT, INPUT_PULLUP);
  pinMode(ENC4_SW, INPUT_PULLUP);

  // ═══════════════ Read initial encoder states continued ═══════════════
  lastClk1 = digitalRead(ENC1_CLK);
  lastClk2 = digitalRead(ENC2_CLK);
  lastClk3 = digitalRead(ENC3_CLK);
  lastClk4 = digitalRead(ENC4_CLK);

  // ═══════════════ 🎭 STARTUP SYMPHONY WITH TRUE HOME - THE SACRED AWAKENING 🎭 ═══════════════
  Serial.println("🎭 Job Security - Awakening Symphony");
  Serial.print("🏗️ MAGNIFICENT DIMENSIONS: ");
  Serial.print(TOTAL_Y_TRAVEL_INCHES);
  Serial.print("\" x ");
  Serial.print(TOTAL_Z_TRAVEL_INCHES);
  Serial.println("\" - A Marvel Of Precision!");
  Serial.println("🎯 Ready to dance...");

  // ═══════════════ All displays light up ═══════════════
  for (int i = 0; i < 4; i++) {
    display1.writeDigitRaw(i, 0xFFFF);
    display2.writeDigitRaw(i, 0xFFFF);
    display3.writeDigitRaw(i, 0xFFFF);
  }
  display1.writeDisplay();
  display2.writeDisplay();
  display3.writeDisplay();

  // ═══════════════ LED Symphony ═══════════════
  for (int i = 0; i < 3; i++) {
    programRing.setPixelColor(0, programRing.Color(255, 0, 0));
    lockinRing.setPixelColor(0, lockinRing.Color(0, 255, 0));
    startRing.setPixelColor(0, startRing.Color(0, 0, 255));
    speedRing.setPixelColor(0, speedRing.Color(255, 255, 0));
    programRing.show();
    lockinRing.show();
    startRing.show();
    speedRing.show();
    delay(200);

    programRing.setPixelColor(0, programRing.Color(0, 255, 0));
    lockinRing.setPixelColor(0, lockinRing.Color(0, 0, 255));
    startRing.setPixelColor(0, startRing.Color(255, 255, 0));
    speedRing.setPixelColor(0, speedRing.Color(255, 0, 255));
    programRing.show();
    lockinRing.show();
    startRing.show();
    speedRing.show();
    delay(200);
  }

  if (false) {
    (void)yBlockUntil;
    (void)zBlockUntil;
    (void)movedToNewHomeThisCycle;
  }

  // ═══════════════ Perform TRUE HOME sequence during symphony ═══════════════
  Serial.println("🏠 Performing TRUE HOME Sequence...");

  performTrueHomeSequence();

  scrollBootMessage();

  // ═══════════════ Final ready message ═══════════════
  display1.clear();
  display1.writeDigitAscii(0, 'L');
  display1.writeDigitAscii(1, 'E');
  display1.writeDigitAscii(2, 'T');
  display1.writeDigitAscii(3, 'S');
  display1.writeDisplay();

  display2.clear();
  display2.writeDigitAscii(0, 'C');
  display2.writeDigitAscii(1, '0');
  display2.writeDigitAscii(2, 'A');
  display2.writeDigitAscii(3, 'T');
  display2.writeDisplay();

  display3.clear();
  display3.writeDigitAscii(0, 'T');
  display3.writeDigitAscii(1, 'H');
  display3.writeDigitAscii(2, 'I');
  display3.writeDigitAscii(3, 'S');
  display3.writeDisplay();
  delay(1000);

  updateAllLEDs();
  // Log button interaction and validate state
  logSystemState("BUTTON_PRESSED");
  validateSystemState();
  updateDisplay();

  Serial.println("🎯 System Ready - All 3 Displays Online");
  Serial.println("🎛️ 4 Encoders Active - Coordinate Tracking Enabled");
  Serial.println("🏠 TRUE HOME Established at (0,0) - Ready for Teressa");
  Serial.println("🔧 Solenoid should be OFF at startup");
  //>Serial.println("🔧 PATCHED: PC replaced with SH + 50% overlap calculation");
  digitalWrite(SOLENOID_PIN, HIGH);  // ═══════════════ Force it OFF ═══════════════
}  // ← ADD THIS CLOSING BRACE FOR THE FOR LOOP

// ═══════════════ MAIN LOOP (SINGLE, CORRECTED) ═══════════════
void loop() {

  unsigned long currentMillis = millis();

  // ═══════════════ TRUE HOME ENDSTOPS - RECOVERABLE PROTECTION WITH FIXED DEBOUNCING ═══════════════
  static bool lastTrueHomeY = true, lastTrueHomeZ = true;
  static int zTopDebounce = 0;      // Start at 0
  static int yRightDebounce = 0;    // Start at 0
  static bool stableZTop = true;    // Stable state tracking
  static bool stableYRight = true;  // Y endstop stable state tracking

  // Debounce Z_TOP_ENDSTOP
  bool zTopRaw = digitalRead(Z_TOP_ENDSTOP);  // Pin 12 ✅
  if (zTopRaw == LOW) {
    zTopDebounce++;
    if (zTopDebounce >= 3) {  // 3 consecutive LOWs
      stableZTop = false;
      zTopDebounce = 3;  // Cap at 3
    }
  } else {
    zTopDebounce--;
    if (zTopDebounce <= 0) {  // 3 consecutive HIGHs
      stableZTop = true;
      zTopDebounce = 0;  // Cap at 0
    }
  }
  bool currentTrueHomeZ = stableZTop;

  // Debounce Y_RIGHT_ENDSTOP
  bool yRightRaw = digitalRead(Y_RIGHT_ENDSTOP);
  if (yRightRaw == LOW) {
    yRightDebounce++;
    if (yRightDebounce >= 3) {
      stableYRight = false;
      yRightDebounce = 3;
    }
  } else {
    yRightDebounce--;
    if (yRightDebounce <= 0) {
      stableYRight = true;
      yRightDebounce = 0;
    }
  }
  bool currentTrueHomeY = stableYRight;

  // ONLY monitor TRUE HOME endstops during NORMAL OPERATION (not during homing)
  if (!endstopState.isHomingInProgress) {

    // Z TRUE HOME - Stop movement but allow recovery
    if (!currentTrueHomeZ && lastTrueHomeZ) {
      debugPrint("🚨 Z TRUE HOME ENDSTOP - STOPPING MOVEMENT (Recoverable)", DEBUG_BASIC);
      //>systemArmed = false;  // Stop automation
      zAxisActive = false;  // Stop Z movement
                            // NO endstopEmergencyTriggered = true - allows recovery
    }

    // Y TRUE HOME - Stop movement but allow recovery
    if (!currentTrueHomeY && lastTrueHomeY) {
      debugPrint("🚨 Y TRUE HOME ENDSTOP - STOPPING MOVEMENT (Recoverable)", DEBUG_BASIC);
      systemArmed = false;  // Stop automation
      yAxisActive = false;  // Stop Y movement
                            // NO endstopEmergencyTriggered = true - allows recovery
    }
  }
  // DURING TRUE HOME SEQUENCE: These endstops are handled gently in performTrueHomeSequence()
  // with proper contact detection and 2mm backoff for precise reference establishment

  lastTrueHomeY = currentTrueHomeY;
  lastTrueHomeZ = currentTrueHomeZ;

  // Handle encoders at high priority
  if (currentMillis - lastMillis >= encoderSampleRate) {
    lastMillis = currentMillis;
    handleAllEncoders();
  }

  // 🔍 Enhanced monitoring
  monitorPositionConsistency();
  //>monitorEndstopStates();

  // ═══════════════ Handle button inputs ═══════════════
  programDebouncer.update();
  lockinDebouncer.update();
  startDebouncer.update();
  speedDebouncer.update();
  performPeriodicSave();  // telemetry only; compile-time controllable

  // FRAM-BASED PRECISION MONITORING SYSTEM
  static unsigned long lastDriftCheck = 0;
  static unsigned long lastStateCheck = 0;
  static unsigned long lastIdleLog = 0;
  static unsigned long lastMemoryReport = 0;

  // Position drift detection every 2 seconds
  if (millis() - lastDriftCheck > 2000) {
    detectAndCorrectDrift();
    lastDriftCheck = millis();
  }

  // State validation every 1 second
  if (millis() - lastStateCheck > 1000) {
    validateSystemState();
    lastStateCheck = millis();
  }

  // Log position during idle periods (when not actively moving)
  if (!yAxisActive && !zAxisActive && toggleTracker.hasMovement && (millis() - lastIdleLog > 5000)) {
    logPositionToFRAM(POS_OP_IDLE_UPDATE);
    toggleTracker.hasMovement = false;
    lastIdleLog = millis();
  }

  // Memory usage report every 60 seconds
  if (millis() - lastMemoryReport > 60000) {
    Serial.println("FRAM MEMORY PALACE STATUS:");
    Serial.print("   Position records: ");
    Serial.print(positionLogIndex);
    Serial.print(" of ");
    Serial.println(MAX_POSITION_HISTORY);
    Serial.print("   State records: ");
    Serial.print(stateLogIndex);
    Serial.print(" of ");
    Serial.println(MAX_STATE_RECORDS);
    Serial.print("   Memory health: ");
    Serial.print(framManager.usagePercent);
    Serial.println("% capacity");
    lastMemoryReport = millis();
  }

  if (programDebouncer.fell()) {
    // ═══════════════ Stop boot/completion message when user interacts ═══════════════
    if (scrollMessage.isActive) {
      scrollMessage.isActive = false;
    }

    // ═══════════════ Only respond if system is not locked ═══════════════
    if (!systemLocked) {
      programState = !programState;
      currentMode = programState ? PROGRAMMING : MANUAL_JOG;
      updateAllLEDs();
      // Log button interaction and validate state
      logSystemState("BUTTON_PRESSED");
      validateSystemState();
      Serial.print("🎛️ Program Mode: ");
      Serial.println(programState ? "🔧 PROGRAMMING - SH & Encoders Active for Programming" : "🎮 MANUAL JOG");
    } else {
      Serial.println("🔒 System locked - PROGRAM button disabled");
    }
  }

  if (lockinDebouncer.fell()) {
    // ═══════════════ Stop boot/completion message when user interacts ═══════════════
    if (scrollMessage.isActive) {
      scrollMessage.isActive = false;
    }

    lockState = !lockState;

    if (lockState) {
      float yInchesFromTrueHome = abs(position.currentY - position.trueHomeY) / Y_STEPS_PER_INCH;
      float zInchesFromTrueHome = abs(position.currentZ - position.trueHomeZ) / Z_STEPS_PER_INCH;
      bool atTrueHome = (yInchesFromTrueHome <= 1.0) && (zInchesFromTrueHome <= 1.0);

      if (atTrueHome) {
        Serial.println("LOCK @ TRUE HOME → System locked, no coordinates saved");
        systemLocked = true;
        currentMode = LOCKED;
      } else {
        Serial.println("LOCK away from TRUE HOME → Saving position and locking system");
        simpleSavePosition();
        systemLocked = true;
        currentMode = LOCKED;
      }
    } else {
      // Unlock logic stays the same
      systemLocked = false;
      currentMode = MANUAL_JOG;
    }

    updateAllLEDs();
    // Log button interaction and validate state
    logSystemState("BUTTON_PRESSED");
    validateSystemState();
    Serial.print("🔒 Lock State: ");
    Serial.println(lockState ? "🔒 PROTECTED MODE" : "🔓 FULL CONTROL");
  }

  if (startDebouncer.fell()) {
    startState = !startState;

    // Only execute logic when START button is pressed AND conditions are met
    if (startState && lockState && programMemory.isValid) {
      Serial.println("🚀 CONDITIONS MET - EXECUTING START BUTTON LOGIC!");
      Serial.println("⚠️ TERESSA - WATCH CLOSELY! E-STOP READY!");

      handleStartButtonLogic();

      // 🔓 Clear lock after execution
      lockState = false;
      systemLocked = false;
      currentMode = MANUAL_JOG;  // Don’t revert to EXECUTING!
      updateAllLEDs();
      // Log button interaction and validate state
      logSystemState("BUTTON_PRESSED");
      validateSystemState();
    } else {
      // Print status when button pressed
      Serial.print("🚀 Start: ");
      Serial.println(startState ? "🟢 ARMED/EXECUTING" : "⚫ STANDBY");
    }
  }

  if (speedDebouncer.fell()) {
    // ═══════════════ Only respond if system is not locked ═══════════════
    if (!systemLocked) {
      macroSpeed = !macroSpeed;
      updateAllLEDs();
      // Log button interaction and validate state
      logSystemState("BUTTON_PRESSED");
      validateSystemState();
      Serial.println(macroSpeed ? "🟣 MACRO - Precise, No Ramping" : "🔴 NORMAL - Standard with Ramping");
    } else {
      Serial.println("🔒 System locked - SPEED button disabled");
      yBlockUntil = millis() + 150;
      zBlockUntil = millis() + 150;
    }
  }

  // Encoder 4 push button solenoid control (momentary)
  if (!systemLocked && !systemArmed) {
    bool enc4Button = digitalRead(ENC4_SW);
    static bool lastEnc4State = HIGH;

    if (enc4Button != lastEnc4State) {
      if (enc4Button == LOW) {
        digitalWrite(SOLENOID_PIN, LOW);  // ON (active LOW)
        Serial.println("Manual solenoid ON");
      } else {
        digitalWrite(SOLENOID_PIN, HIGH);  // OFF
        Serial.println("Manual solenoid OFF");
      }
      lastEnc4State = enc4Button;
    }
  }

  // ═══════════════ TOGGLE SWITCH DEBOUNCING (MOTOR CONTROL ONLY) ═══════════════
  // We require N consecutive identical samples (every 5ms) before accepting a flip.
  // This kills “one-sample nibble” pulses that cause micro-jitter.
  static unsigned long lastToggleChange = 0;

  // Stable states used by motor control
  static bool stableToggleYLeft = true;
  static bool stableToggleYRight = true;
  static bool stableToggleZUp = true;
  static bool stableToggleZDown = true;

  // Candidate states and counters for consecutive-sample debounce
  static bool candYLeft = true, candYRight = true, candZUp = true, candZDown = true;
  static uint8_t cntYLeft = 3, cntYRight = 3, cntZUp = 3, cntZDown = 3;

  static unsigned long lastToggleSample = 0;
  const unsigned long TOGGLE_SAMPLE_MS = 5;  // sample every 5ms
  const uint8_t TOGGLE_CONSEC_REQUIRED = 3;  // need 3 consecutive identical samples (~15ms)

  // Read RAW toggle switch states (ACTIVE-LOW hardware with INPUT_PULLUP)
  bool yLeftToggleRaw = digitalRead(Y_LEFT);    // Pin 10 - Left toggle
  bool yRightToggleRaw = digitalRead(Y_RIGHT);  // Pin 11 - Right toggle
  bool zUpToggleRaw = digitalRead(Z_UP);        // Pin 40 - Up toggle
  bool zDownToggleRaw = digitalRead(Z_DOWN);    // Pin 41 - Down toggle

  if (millis() - lastToggleSample >= TOGGLE_SAMPLE_MS) {
    lastToggleSample = millis();

    // Y_LEFT
    if (yLeftToggleRaw == candYLeft) {
      cntYLeft++;
    } else {
      candYLeft = yLeftToggleRaw;
      cntYLeft = 1;
    }
    if (cntYLeft >= TOGGLE_CONSEC_REQUIRED && stableToggleYLeft != candYLeft) {
      // On release (LOW -> HIGH), start short deadtime to kill post-release nibble
      if (stableToggleYLeft == LOW && candYLeft == HIGH) { yBlockUntil = millis() + TOGGLE_RELEASE_DEADTIME_MS; }
      stableToggleYLeft = candYLeft;
    }

    // Y_RIGHT
    if (yRightToggleRaw == candYRight) {
      cntYRight++;
    } else {
      candYRight = yRightToggleRaw;
      cntYRight = 1;
    }
    if (cntYRight >= TOGGLE_CONSEC_REQUIRED && stableToggleYRight != candYRight) {
      if (stableToggleYRight == LOW && candYRight == HIGH) { yBlockUntil = millis() + TOGGLE_RELEASE_DEADTIME_MS; }
      stableToggleYRight = candYRight;
    }

    // Z_UP
    if (zUpToggleRaw == candZUp) {
      cntZUp++;
    } else {
      candZUp = zUpToggleRaw;
      cntZUp = 1;
    }
    if (cntZUp >= TOGGLE_CONSEC_REQUIRED && stableToggleZUp != candZUp) {
      if (stableToggleZUp == LOW && candZUp == HIGH) { zBlockUntil = millis() + TOGGLE_RELEASE_DEADTIME_MS; }
      stableToggleZUp = candZUp;
    }

    // Z_DOWN
    if (zDownToggleRaw == candZDown) {
      cntZDown++;
    } else {
      candZDown = zDownToggleRaw;
      cntZDown = 1;
    }
    if (cntZDown >= TOGGLE_CONSEC_REQUIRED && stableToggleZDown != candZDown) {
      if (stableToggleZDown == LOW && candZDown == HIGH) { zBlockUntil = millis() + TOGGLE_RELEASE_DEADTIME_MS; }
      stableToggleZDown = candZDown;
    }
  }

  // Use these stable values for motor control
  bool yLeftNow = stableToggleYLeft;
  bool yRightNow = stableToggleYRight;
  bool zUpNow = stableToggleZUp;
  bool zDownNow = stableToggleZDown;

  (void)lastToggleChange;  // silence unused-but-set warning (keeps debug hook)


  // Log any toggle activity
  //>static bool lastLoggedYLeft = true, lastLoggedYRight = true;
  //>static bool lastLoggedZUp = true, lastLoggedZDown = true;

  // DEBUGGING IN SERIAL MONITOR FOR Y AND Z CORORDANATES
  //>if (yLeftNow != lastLoggedYLeft || yRightNow != lastLoggedYRight || zUpNow != lastLoggedZUp || zDownNow != lastLoggedZDown) {
  //>Serial.print("🎮 TOGGLE STATE: YL=");
  //>Serial.print(yLeftNow);
  //>Serial.print(", YR=");
  //>Serial.print(yRightNow);
  //>Serial.print(", ZU=");
  //>Serial.print(zUpNow);
  //>Serial.print(", ZD=");
  //>Serial.println(zDownNow);

  //>lastLoggedYLeft = yLeftNow;
  //>lastLoggedYRight = yRightNow;
  //>lastLoggedZUp = zUpNow;
  //>lastLoggedZDown = zDownNow;

  // ═══════════════ Handle motor control with mutual exclusion - only when not armed AND not locked ═══════════════
  if (!systemArmed && !systemLocked) {
    // ═══════════════ Y AXIS ═══════════════
    bool yRequested = !yLeftNow || !yRightNow;

    // ═══════════════ Z AXIS ═══════════════
    bool zRequested = !zUpNow || !zDownNow;

    // ═══════════════ Apply mutual exclusion ═══════════════
    if (isSafeToMove(yRequested, zRequested)) {

      // SIMPLE ENDSTOP DISABLE SYSTEM - MANUAL MODE ONLY
      if (!systemArmed && currentMode == MANUAL_JOG) {

        // Check if we should re-enable endstops (check both Y and Z separately)
        if (endstopsDisabled && stepsSinceDisabled >= Y_STEPS_TO_REENABLE && stepsSinceDisabled >= Z_STEPS_TO_REENABLE) {
          endstopsDisabled = false;
          stepsSinceDisabled = 0;
          Serial.println("Endstops re-enabled after sufficient steps");
        }

        // Only check endstops if they're enabled
        if (!endstopsDisabled) {
          if ((yRequested && digitalRead(Y_RIGHT_ENDSTOP) == LOW) || (zRequested && digitalRead(Z_TOP_ENDSTOP) == LOW)) {
            Serial.println("Endstop hit - DISABLING until Y=500 and Z=5000 steps");
            endstopsDisabled = true;
            stepsSinceDisabled = 0;
            delay(500);  // Force 0.5 second pause
          }
        }
      }
    }

    // ═══════════════ Y AXIS CONTROL - SIMPLIFIED FOR SMOOTH RELIABLE MOVEMENT ═══════════════
    if (yRequested) {
      // ── Handle direction setting
      if (yLeftPrev && !yLeftNow) {           // moved to LEFT
        digitalWrite(Y_DIR, LOW);             // LEFT=LOW
      } else if (yRightPrev && !yRightNow) {  // moved to RIGHT
        digitalWrite(Y_DIR, HIGH);            // RIGHT=HIGH
      }

      // ── Simple, reliable stepping - NO complex ramping, NO blocking loops
      static unsigned long lastYStepTime = 0;
      if ((!yLeftNow || !yRightNow) && (millis() - lastYStepTime >= 1)) {

        // Simple deadtime gate
        if (millis() >= yBlockUntil) {
          int delayTime;

          // Clean speed selection - no ramping conflicts
          if (macroSpeed) {
            delayTime = 2000;  // MACRO (Purple) - slow and precise
          } else {
            delayTime = 200;  // NORMAL (Red) - fast and smooth
          }

          // Single Y step pulse - reliable timing
          digitalWrite(Y_STEP, HIGH);
          delayMicroseconds(50);  // Increased pulse width for reliability
          digitalWrite(Y_STEP, LOW);
          delayMicroseconds(delayTime);
          trackMotorMovement(Y_STEP, digitalRead(Y_DIR));

          // Update timing
          lastYStepTime = millis();
          lastMotorAction = millis();
        }
      }

      // ── Simple release handling - NO blocking while() loops
      if (!yLeftPrev && yLeftNow) {                           // LEFT released
        yBlockUntil = millis() + TOGGLE_RELEASE_DEADTIME_MS;  // Simple deadtime
      } else if (!yRightPrev && yRightNow) {                  // RIGHT released
        yBlockUntil = millis() + TOGGLE_RELEASE_DEADTIME_MS;  // Simple deadtime
      }
    }  // ✅ end Y axis control

    // ═══════════════ Z AXIS CONTROL (RED = TRUE-HOME-LIKE RAMP, MACRO = SLOW FIXED) ═══════════════
    static bool zJogActive = false;
    static bool zJogDirUp = true;  // true=UP, false=DOWN
    static unsigned long zJogStartUs = 0;

    if (zRequested) {
      // Active-low toggles (in your sampler, *_Now is LOW when pressed)
      bool upHeld = !zUpNow;
      bool downHeld = !zDownNow;
      bool anyHeld = upHeld || downHeld;

      // Start/restart ramp on new press or direction change
      if (anyHeld) {
        // If both are held, keep the last direction to avoid oscillation
        bool wantUp = (upHeld && !downHeld)   ? true
                      : (!upHeld && downHeld) ? false
                                              : zJogDirUp;

        if (!zJogActive || (wantUp != zJogDirUp)) {
          zJogActive = true;
          zJogDirUp = wantUp;
          zJogStartUs = micros();
          // Serial.println(zJogDirUp ? "⬆️ Z TOGGLE RAMP (RED): UP" : "⬇️ Z TOGGLE RAMP (RED): DOWN");
        }
      } else {
        // Released → reset ramp state
        zJogActive = false;
      }

      if (anyHeld && (millis() >= zBlockUntil)) {
        int dUs;

        if (!macroSpeed) {
          // RED (Normal) → replicate TRUE HOME ramp: ~1s easing from 50µs → 10µs, then cruise
          unsigned long elapsed = micros() - zJogStartUs;
          if (elapsed < 1000000UL) {
            float progress = (float)elapsed / 1000000.0f;  // 0.0 → 1.0 over ~1s
            dUs = (int)(50.0f - (progress * 40.0f));       // 50 → 10
            if (dUs < 10) dUs = 10;
          } else {
            dUs = 10;  // cruise delay (µs)
          }
        } else {
          // MACRO (Purple) → slow fixed stepping
          dUs = 100;
        }

        // One shaped step
        digitalWrite(Z_DIR, zJogDirUp ? HIGH : LOW);  // +Z = UP on HIGH
        digitalWrite(Z_STEP, HIGH);
        delayMicroseconds(Z_PULSE_WIDTH);
        digitalWrite(Z_STEP, LOW);
        delayMicroseconds(dUs);

        // Book-keeping consistent with your tracker
        trackMotorMovement(Z_STEP, zJogDirUp ? HIGH : LOW);
        lastMotorAction = millis();
      }  // ✅ end Z axis control
    }
  }
}

// ═══════════════ COMPLETE 4-ENCODER HANDLING ═══════════════
void handleAllEncoders() {
  // ═══════════════ Only respond to encoders if system is NOT locked AND PROGRAM mode is active ═══════════════
  if (systemLocked || !programState) {
    return;  // ═══════════════ Encoders disabled when system is locked OR when PROGRAM mode is OFF ═══════════════
  }

  unsigned long now = millis();

  // ═══════════════ ENCODER 1 ═══════════════
  int clk1 = digitalRead(ENC1_CLK);
  if (clk1 != lastClk1 && clk1 == LOW && (now - lastDebounceTime1 > debounceDelay)) {
    lastDebounceTime1 = now;
    delayMicroseconds(1000);
    int dt1 = digitalRead(ENC1_DT);

    if (state.encoder1Mode == SystemState::SH_MODE) {
      if (dt1 != clk1) state.sprayHeightInches++;
      else state.sprayHeightInches--;
      state.sprayHeightInches = constrain(state.sprayHeightInches, 1, 99);
    } else {
      if (dt1 != clk1) state.coveragePercent += 5;
      else state.coveragePercent -= 5;
      state.coveragePercent = constrain(state.coveragePercent, 0, 95);
    }
    updateDisplay();
  }
  lastClk1 = clk1;

  // ═══════════════ Encoder 1 button ═══════════════
  bool btn1 = digitalRead(ENC1_SW);
  if (!btn1 && !buttonLatched1 && (now - lastDebounceTime1 > debounceDelay)) {
    buttonLatched1 = true;
    lastDebounceTime1 = now;

    if (state.encoder1Mode == SystemState::SH_MODE) {
      state.encoder1Mode = SystemState::CP_MODE;
    } else {
      state.encoder1Mode = SystemState::SH_MODE;
    }
    updateDisplay();
  }
  if (btn1 && buttonLatched1) buttonLatched1 = false;

  // ═══════════════ ENCODER 2 ═══════════════
  int clk2 = digitalRead(ENC2_CLK);
  if (clk2 != lastClk2 && clk2 == LOW && (now - lastDebounceTime2 > debounceDelay)) {
    lastDebounceTime2 = now;
    delayMicroseconds(1000);
    int dt2 = digitalRead(ENC2_DT);

    if (state.encoder2Mode == SystemState::RC_MODE) {
      if (dt2 != clk2) state.repeatCount++;
      else state.repeatCount--;
      state.repeatCount = constrain(state.repeatCount, 0, 99);  // ═══════════════ RC must have 0! ═══════════════
    } else {
      if (dt2 != clk2) state.velocityScale++;
      else state.velocityScale--;
      if (state.velocityScale > 10) state.velocityScale = 10;
      if (state.velocityScale < 0) state.velocityScale = 0;
      state.velocityScale = constrain(state.velocityScale, 0, 10);
    }
    updateDisplay();
  }
  lastClk2 = clk2;

  // ═══════════════ Encoder 2 button ═══════════════
  bool btn2 = digitalRead(ENC2_SW);
  if (!btn2 && !buttonLatched2 && (now - lastDebounceTime2 > debounceDelay)) {
    buttonLatched2 = true;
    lastDebounceTime2 = now;

    if (state.encoder2Mode == SystemState::RC_MODE) {
      state.encoder2Mode = SystemState::V_PLUS_MODE;
    } else {
      state.encoder2Mode = SystemState::RC_MODE;
    }
    updateDisplay();
  }
  if (btn2 && buttonLatched2) buttonLatched2 = false;

  // ═══════════════ ENCODER 3 - Area Height ═══════════════
  int clk3 = digitalRead(ENC3_CLK);
  if (clk3 != lastClk3 && clk3 == LOW && (now - lastDebounceTime3 > debounceDelay)) {
    lastDebounceTime3 = now;
    delayMicroseconds(1000);
    int dt3 = digitalRead(ENC3_DT);

    if (dt3 != clk3) state.areaHeight++;
    else state.areaHeight--;
    state.areaHeight = constrain(state.areaHeight, 1, 99);

    state.showingWidth = false;
    Serial.print("📏 Area Height: ");
    Serial.print(state.areaHeight);
    Serial.println("in");
    updateDisplay();
  }
  lastClk3 = clk3;

  // ═══════════════ ENCODER 4 - Area Width ═══════════════
  int clk4 = digitalRead(ENC4_CLK);
  if (clk4 != lastClk4 && clk4 == LOW && (now - lastDebounceTime4 > debounceDelay)) {
    lastDebounceTime4 = now;
    delayMicroseconds(1000);
    int dt4 = digitalRead(ENC4_DT);

    if (dt4 != clk4) state.areaWidth++;
    else state.areaWidth--;
    state.areaWidth = constrain(state.areaWidth, 1, 99);

    state.showingWidth = true;
    Serial.print("📏 Area Width: ");
    Serial.print(state.areaWidth);
    Serial.println("in");
    updateDisplay();
  }
  lastClk4 = clk4;
}

// ═══════════════ MOTOR PULSE FUNCTIONS ═══════════════
void pulseMotor(int pin) {
  int delayTime = macroSpeed ? 600 : 200;  // ═══════════════ MACRO=precise/slow, NORMAL=fast ═══════════════
  digitalWrite(pin, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(pin, LOW);
  delayMicroseconds(delayTime);
}

// ═══════════════ COMPLETE DISPLAY UPDATE ═══════════════
void updateDisplay() {
  // ═══════════════ Display 1: SH/ZD ═══════════════
  display1.clear();

  if (state.encoder1Mode == SystemState::SH_MODE) {
    display1.writeDigitAscii(0, 'S');
    display1.writeDigitAscii(1, 'H');
    int tens = state.sprayHeightInches / 10;
    int ones = state.sprayHeightInches % 10;
    display1.writeDigitAscii(2, '0' + tens);
    display1.writeDigitAscii(3, '0' + ones);
  } else {
    display1.writeDigitAscii(0, 'C');
    display1.writeDigitAscii(1, 'P');
    int tens = state.coveragePercent / 10;
    int ones = state.coveragePercent % 10;
    display1.writeDigitAscii(2, '0' + tens);
    display1.writeDigitAscii(3, '0' + ones);
  }
  display1.writeDisplay();

  // ═══════════════ Display 2: RC/V+ ═══════════════
  display2.clear();
  int tens, ones;

  if (state.encoder2Mode == SystemState::RC_MODE) {
    display2.writeDigitAscii(0, 'R');
    display2.writeDigitAscii(1, 'C');
    tens = state.repeatCount / 10;
    ones = state.repeatCount % 10;
    if (tens > 0) display2.writeDigitAscii(2, '0' + tens);
    else display2.writeDigitAscii(2, '0');
    display2.writeDigitAscii(3, '0' + ones);
  } else if (state.encoder2Mode == SystemState::V_PLUS_MODE) {
    display2.writeDigitAscii(0, 'V');
    display2.writeDigitAscii(1, '+');
    tens = state.velocityScale / 10;
    ones = state.velocityScale % 10;
    if (tens > 0) display2.writeDigitAscii(2, '0' + tens);
    else display2.writeDigitAscii(2, '0');
    display2.writeDigitAscii(3, '0' + ones);
  }
  display2.writeDisplay();

  // ═══════════════ Display 3: AW/AH ═══════════════
  display3.clear();

  if (state.showingWidth) {
    display3.writeDigitAscii(0, 'A');
    display3.writeDigitAscii(1, 'W');
    tens = state.areaWidth / 10;
    ones = state.areaWidth % 10;
    display3.writeDigitAscii(2, '0' + tens);
    display3.writeDigitAscii(3, '0' + ones);
  } else {
    display3.writeDigitAscii(0, 'A');
    display3.writeDigitAscii(1, 'H');
    tens = state.areaHeight / 10;
    ones = state.areaHeight % 10;
    display3.writeDigitAscii(2, '0' + tens);
    display3.writeDigitAscii(3, '0' + ones);
  }
  display3.writeDisplay();
}

// ═══════════════ LED UPDATES - LOGICAL SPEED NAMING AND COLORS + LOCK INDICATION ═══════════════
void updateAllLEDs() {

  // 🔴 PROGRAM LED - Amber when programming mode is active
  uint32_t programColor;
  if (programState) {
    programColor = programRing.Color(255, 127, 0);  // Amber = Programming active
  } else {
    programColor = programRing.Color(0, 0, 0);  // Off = Manual jog mode
  }
  programRing.setPixelColor(0, programColor);

  // 🔵 LOCK LED - Simple states with TRUE HOME indication
  uint32_t lockColor;
  if (lockState && systemLocked) {
    // Lock button pressed AND at TRUE HOME
    lockColor = lockinRing.Color(139, 0, 0);  // Deep Red = Armed at TRUE HOME
  } else if (lockState) {
    // Lock button pressed but not at TRUE HOME
    lockColor = lockinRing.Color(255, 20, 147);  // Pink = Armed/Ready to execute
  } else {
    // Normal unlocked state
    lockColor = lockinRing.Color(0, 0, 0);  // Off = Normal operation
  }
  lockinRing.setPixelColor(0, lockColor);

  // 🟢 START LED — double pulse then solid green
  {
    const bool ready = (lockState && programMemory.isValid);
    const bool useBreathing = featureFlags.isEnabled(FEATURE_LED_BREATHING);

    // Track transitions into READY
    static bool wasReady = false;
    static uint8_t phase = 255;  // 0..3 = pulsing, 255 = idle/solid
    static unsigned long t0 = 0;

    if (ready && !wasReady) {  // just entered READY
      phase = 0;               // start pulse sequence
      t0 = millis();
    }
    wasReady = ready;

    if (ready) {
      unsigned long now = millis();
      switch (phase) {
        case 0:  // ON #1 (120ms)
          startRing.setPixelColor(0, 0, 255, 0);
          if (now - t0 > 120) {
            t0 = now;
            phase = 1;
          }
          break;
        case 1:  // OFF (90ms)
          startRing.setPixelColor(0, 0, 0, 0);
          if (now - t0 > 90) {
            t0 = now;
            phase = 2;
          }
          break;
        case 2:  // ON #2 (120ms)
          startRing.setPixelColor(0, 0, 255, 0);
          if (now - t0 > 120) {
            t0 = now;
            phase = 3;
          }
          break;
        case 3:  // OFF (250ms)
          startRing.setPixelColor(0, 0, 0, 0);
          if (now - t0 > 250) { phase = 255; }  // sequence complete
          break;
        default:  // Breathing or solid green
          if (useBreathing) {
            int brightness = 128 + 127 * sin(millis() * 0.003);
            startRing.setPixelColor(0, 0, brightness, 0);
          } else {
            startRing.setPixelColor(0, 0, 255, 0);
          }
          break;
      }
    } else {
      // Not ready → LED off
      startRing.setPixelColor(0, 0, 0, 0);
    }
  }

  // 🟡 SPEED LED - Clear speed indication
  uint32_t speedColor;
  if (systemLocked) {
    speedColor = speedRing.Color(0, 0, 0);  // Off = Disabled when locked
  } else if (macroSpeed) {
    speedColor = speedRing.Color(138, 43, 226);  // Purple = MACRO (slow/precise)
  } else {
    speedColor = speedRing.Color(255, 0, 0);  // Red = NORMAL (fast with ramping)
  }
  speedRing.setPixelColor(0, speedColor);

  // Apply all changes
  programRing.show();
  lockinRing.show();
  startRing.show();
  speedRing.show();
}

// MOTOR MOVEMENT TRACKING WITH SMART FREQUENCY LOGGING
void trackMotorMovement(int pin, bool direction) {
  // Serial Monitor Tracking of Steps
  //>Serial.print("TRACK: ");
  //>Serial.print(pin == Y_STEP ? "Y" : "Z");
  //>Serial.print(direction == HIGH ? "+" : "-");
  //>Serial.print(" Before: Y=");
  //>Serial.print(position.currentY);
  //>Serial.print(" Z=");
  //>Serial.println(position.currentZ);

  // Simple drift check only - no FRAM logging during movement
  static int stepCount = 0;
  if (++stepCount >= 100) {
    long yDrift = abs(position.currentY - toggleTracker.lastRecordedY);
    long zDrift = abs(position.currentZ - toggleTracker.lastRecordedZ);

    if (yDrift > 10 || zDrift > 10) {
      Serial.println("DRIFT CORRECTED");
      toggleTracker.lastRecordedY = position.currentY;
      toggleTracker.lastRecordedZ = position.currentZ;
    }
    stepCount = 0;
  }

  // Periodic coordinate reporting
  static unsigned long lastCoordReport = 0;
  if (millis() - lastCoordReport > 2000) {
    Serial.print("Current Position: Y=");
    Serial.print(position.currentY);
    Serial.print(", Z=");
    Serial.println(position.currentZ);
    lastCoordReport = millis();
  }

  // Count steps for endstop re-enable
  if (endstopsDisabled) {
    stepsSinceDisabled++;
  }

  if (pin == Y_STEP) {
    if (direction == HIGH) position.currentY++;
    else position.currentY--;
    toggleTracker.lastRecordedY = position.currentY;
  } else if (pin == Z_STEP) {
    if (direction == HIGH) position.currentZ++;
    else position.currentZ--;
    toggleTracker.lastRecordedZ = position.currentZ;
  }

  toggleTracker.hasMovement = true;
}

//╔══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╗
//║ 💃 JOB SECURITY v18.3 - FINAL COMPLETE MASTERPIECE 💃
//║ 🎭 "Grace in Motion" - The Sacred Dance Complete 🎭
//║ ✨ Built by ChatGPTo4 / 5, Claude Sonnet 4, Monday, ChatGPT4.1 API, James Lee and John Thomas DuCrest Lock - A Sacred Creation ✨
//║ 🎨 HPLV Gun Controller - Area Defined Spray Perfection 🎨
//║ 🏠 TRUE HOME & NEW HOME - Precision Memory System - Manual and Atomic FRAM 🏠
//║ 🏛️ 66" x 36" Cathedral of Precision - Ready to Dance 🏛️
//║
//║ 🛠️ ENHANCEMENTS APPLIED:
//║ • Enhanced FRAM with connectivity testing & checksums
//║ • Custom made level shifter for the 7 segment displays
//║ • Added parameter validation (minimum values)
//║ • Enhanced display error recovery
//║ • Centralized ramping calculations
//║ • Added position sanity checking
//║ • Improved FRAM reade behavior during reads - Enhanced deletion of old FRAM data
//║ • Confirmed Teensy 4.1 compatibility (Pin 0 & Wire are perfect!)
//║ • Added non-blocking movement with pause control
//║ • Intelligent dual purpose End Stops
//║ • Enhanced operator monitoring and safety features
//║
//║ 🎉 SHE'S READY TO DANCE! 🎉
//╚═══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╝
