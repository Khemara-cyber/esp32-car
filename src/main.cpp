
#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include <Preferences.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

// Private and git-ignored; without it Car 3 runs only its own SWARM-X network.
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#define HOME_WIFI_SSID ""
#define HOME_WIFI_PASSWORD ""
#endif

// Flash the same firmware to every car and change only this ID.
// 1 = Explorer, 2 = Mapper / Observer, 3 = Coordinator / Support
#define ROBOT_ID 2

#if ROBOT_ID < 1 || ROBOT_ID > 3
#error ROBOT_ID must be 1, 2, or 3
#endif

// ESP32 to L298N motor pins
const int ENA = 14;
const int IN1 = 27;
const int IN2 = 26;

const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;

// Settings
const int RADIO_CHANNEL = 1;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t CONTROL_TIMEOUT_MS = 800;
const uint32_t RADIO_INTERVAL_MS = 100;
const uint32_t TELEMETRY_INTERVAL_MS = 500;
const uint32_t TEAM_TELEMETRY_TIMEOUT_MS = TELEMETRY_INTERVAL_MS * 3;
// Missions stop when the operator page has been silent this long.
const uint32_t OPERATOR_TIMEOUT_MS = 2000;

// Car 3 exposes this network for the operator while also joining home Wi-Fi.
const char *AP_NAME = "SWARM-X";
const char *AP_PASSWORD = "swarmx12345";

// ESP-NOW
const uint32_t PACKET_MAGIC = 0x53574152;

uint8_t broadcastMac[6] = {
    0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF};

// Car 3's dashboard. ESP-IDF's server handles several connections at once;
// Arduino's WebServer stalls up to 5 s on an idle connection that Chrome
// opens in advance, which made every drive command time out.
httpd_handle_t httpServer = nullptr;

const int BATTERY_PIN = 34;
const int ULTRASONIC_TRIG_PIN = 18;
const int ULTRASONIC_ECHO_PIN = 19;
const int LINE_LEFT_PIN = 4;
const int LINE_RIGHT_PIN = 15;

const bool ULTRASONIC_SERIAL_TEST = false;
// Set true only after installing and calibrating a battery voltage divider.
const bool BATTERY_MONITOR_ENABLED = false;
const float BATTERY_DIVIDER_RATIO = 3.0f;
const float BATTERY_LOW_VOLTAGE = 10.5f;
// Conservative Explorer settings for a fixed, front-facing ultrasonic sensor.
// Stop before the requested 20-30 cm range to leave braking margin.
const int OBSTACLE_DISTANCE_CM = 35;
const int OBSTACLE_CLEAR_DISTANCE_CM = 60;
// readObstacleDistanceCm() returns this when no echo comes back in time.
const int ULTRASONIC_NO_ECHO_CM = 999;
const int ULTRASONIC_MAX_RANGE_CM = 400;
const int AVOIDANCE_REVERSE_SPEED = 90;
const int AVOIDANCE_TURN_SPEED = 95;
const uint32_t AVOIDANCE_STOP_MS = 250;
const uint32_t AVOIDANCE_REVERSE_MS = 350;
const uint32_t AVOIDANCE_SETTLE_MS = 200;
// Calibrate this on the real floor so this is close to a 90-degree turn.
const uint32_t AVOIDANCE_TURN_90_MS = 360;
const int EXPLORE_SPEED = 165;
// Car 3 follows Car 1 in Explore mode. Calibrate on the real floor:
// FOLLOW_MIN_SPEED must be enough to get the car moving.
const int FOLLOW_TARGET_CM = 50;
const int FOLLOW_TOO_CLOSE_CM = 30;
const int FOLLOW_LOST_CM = 120;
const int FOLLOW_MIN_SPEED = 100;
// Faster than Car 1, or a slightly slower motor would let the gap grow until
// Car 3 loses sight of it.
const int FOLLOW_MAX_SPEED = EXPLORE_SPEED + 30;
const int FOLLOW_BACKOFF_SPEED = 90;
const uint32_t MOTOR_RAMP_INTERVAL_MS = 20;
const int MOTOR_ACCEL_STEP = 5;
const int MOTOR_BRAKE_STEP = 20;

enum RobotMode
{
  MODE_MANUAL = 0,
  MODE_AVOID = 1,
  MODE_IDLE = 2,
  MODE_LINEFOLLOW = 3,
  MODE_EXPLORE = 4,
  MODE_MAP = 5,
  MODE_COORDINATE = 6,
  // Set only by Car 3: Car 1 turns 180 degrees back the way it came.
  MODE_TURNAROUND = 7
};

enum RobotRole
{
  ROLE_EXPLORER = 1,
  ROLE_MAPPER = 2,
  ROLE_COORDINATOR = 3
};

const RobotRole ROBOT_ROLE = static_cast<RobotRole>(ROBOT_ID);
const bool IS_COORDINATOR = ROBOT_ROLE == ROLE_COORDINATOR;

enum PacketType
{
  PACKET_COMMAND = 1,
  PACKET_TELEMETRY = 2,
  PACKET_SCAN = 3
};

enum AvoidancePhase
{
  AVOIDANCE_READY,
  AVOIDANCE_STOPPING,
  AVOIDANCE_REVERSING,
  AVOIDANCE_SCAN_LEFT,
  AVOIDANCE_READ_LEFT,
  AVOIDANCE_SCAN_RIGHT,
  AVOIDANCE_READ_RIGHT,
  AVOIDANCE_CHOOSE_PATH,
  AVOIDANCE_LOCKED
};

volatile RobotMode currentMode = MODE_MANUAL;
float batteryVoltage = 0.0f;
int lastObstacleDistanceCm = 999;
int lastLineTurn = 0;
AvoidancePhase avoidancePhase = AVOIDANCE_READY;
uint32_t avoidancePhaseStartedAt = 0;
int leftScanDistanceCm = 0;
int rightScanDistanceCm = 0;
// False until the first real echo; an unplugged sensor never sets it.
bool ultrasonicHasEchoed = false;

// ESP-NOW protocol. Commands originate only from Car 3; every car broadcasts
// telemetry so the coordinator can see the team's observations.
struct __attribute__((packed)) PacketHeader
{
  uint32_t magic;
  uint8_t type;
  uint8_t source;
};

struct __attribute__((packed)) DrivePacket
{
  uint32_t magic;
  uint8_t type;
  uint8_t source;
  int16_t left;
  int16_t right;
  uint8_t mode;
  uint8_t modeCommandId; // changes on every mode button press, even a repeat
};

struct __attribute__((packed)) TelemetryPacket
{
  uint32_t magic;
  uint8_t type;
  uint8_t source;
  uint8_t role;
  uint16_t distanceCm;
  uint8_t blocked;
  uint32_t uptimeMs;
};

// Car 2's surroundings scan. Directions are relative to Car 2's heading when
// the scan started, turning right: front, right, back, left.
const int OBSERVER_SCAN_STEPS = 4;

struct __attribute__((packed)) ScanPacket
{
  uint32_t magic;
  uint8_t type;
  uint8_t source;
  uint8_t scanId;   // changes with every new scan
  uint8_t readings; // directions measured so far, 0-4
  uint16_t distanceCm[OBSERVER_SCAN_STEPS];
};

// Motor variables
int16_t targetLeft = 0;
int16_t targetRight = 0;

int16_t appliedLeft = 0;
int16_t appliedRight = 0;

volatile int16_t receivedLeft = 0;
volatile int16_t receivedRight = 0;
volatile RobotMode receivedMode = MODE_IDLE;
volatile uint8_t receivedModeCommandId = 0;
uint8_t modeCommandId = 0;
volatile uint32_t receivedAt = 0;

struct TeamObservation
{
  uint16_t distanceCm;
  bool blocked;
  uint32_t receivedAt;
};

TeamObservation teamObservations[4] = {};

// Car 2: its own scan in progress. Car 3: the last scan received from Car 2.
enum ObserverPhase
{
  OBSERVER_IDLE,
  OBSERVER_SETTLE,
  OBSERVER_TURN,
  OBSERVER_DONE
};

ObserverPhase observerPhase = OBSERVER_IDLE;
uint32_t observerPhaseStartedAt = 0;
uint8_t observerScanId = 0;
uint8_t observerReadings = 0;
uint16_t observerDistanceCm[OBSERVER_SCAN_STEPS] = {};

ScanPacket teamScan = {};
uint32_t teamScanReceivedAt = 0;

// Car 1: the 180-degree turn-back ordered by Car 3.
enum TurnaroundPhase
{
  TURNAROUND_NONE,
  TURNAROUND_TURNING,
  TURNAROUND_DONE
};

TurnaroundPhase turnaroundPhase = TURNAROUND_NONE;
uint32_t turnaroundStartedAt = 0;

// Car 3: recovery bookkeeping for a blocked explorer.
const int MAX_EXPLORER_RECOVERIES = 2;
const uint32_t EXPLORER_PROGRESS_MS = 8000;
// Turn time plus margin for the command to arrive and the car to settle.
const uint32_t TURNAROUND_RESUME_MS = AVOIDANCE_TURN_90_MS * 2 + 800;

int explorerRecoveries = 0;
bool explorerNeedsOperator = false;
uint32_t coordinatorPhaseStartedAt = 0;

// Web handlers run in the HTTP server's task: they only leave commands here,
// under webMux, and loop() applies them.
portMUX_TYPE webMux = portMUX_INITIALIZER_UNLOCKED;
int16_t webLeft = 0;
int16_t webRight = 0;
uint32_t webCommandAt = 0; // last /drive or /mode from the page
int pendingMode = -1;      // mode chosen on the page, -1 = none waiting

uint32_t lastRadioSend = 0;
uint32_t lastTelemetrySend = 0;

portMUX_TYPE receiveMux = portMUX_INITIALIZER_UNLOCKED;

// Motor control
void setMotor(int inA, int inB, int en, int speed)
{

  speed = constrain(speed, -255, 255);

  analogWrite(en, 0);

  if (speed == 0)
  {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    return;
  }

  if (speed > 0)
  {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  }
  else
  {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }

  analogWrite(en, abs(speed));
}

// Drive both motors
void drive(int left, int right)
{
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  static uint32_t lastRampUpdate = 0;
  uint32_t now = millis();
  if (now - lastRampUpdate < MOTOR_RAMP_INTERVAL_MS)
  {
    return;
  }

  lastRampUpdate = now;

  auto rampMotor = [](int current, int target)
  {
    if (current != 0 && target != 0 && ((current < 0) != (target < 0)))
    {
      target = 0;
    }

    int step = abs(target) < abs(current) ? MOTOR_BRAKE_STEP : MOTOR_ACCEL_STEP;
    if (current < target)
    {
      return min(current + step, target);
    }
    if (current > target)
    {
      return max(current - step, target);
    }
    return current;
  };

  int16_t nextLeft = rampMotor(appliedLeft, left);
  int16_t nextRight = rampMotor(appliedRight, right);

  if (nextLeft == appliedLeft && nextRight == appliedRight)
  {
    return;
  }

  setMotor(IN1, IN2, ENA, nextLeft);
  setMotor(IN3, IN4, ENB, nextRight);

  appliedLeft = nextLeft;
  appliedRight = nextRight;
}

void driveImmediately(int left, int right)
{
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  setMotor(IN1, IN2, ENA, left);
  setMotor(IN3, IN4, ENB, right);

  appliedLeft = left;
  appliedRight = right;
}

// Receive ESP-NOW messages
void processPacket(const uint8_t *data, int length)
{
  if (length < static_cast<int>(sizeof(PacketHeader)))
  {
    return;
  }

  PacketHeader header;
  memcpy(&header, data, sizeof(header));

  if (header.magic != PACKET_MAGIC || header.source < 1 || header.source > 3)
  {
    return;
  }

  if (header.type == PACKET_COMMAND &&
      !IS_COORDINATOR &&
      header.source == ROLE_COORDINATOR &&
      length == static_cast<int>(sizeof(DrivePacket)))
  {
    DrivePacket packet;
    memcpy(&packet, data, sizeof(packet));

    if (packet.mode > MODE_TURNAROUND)
    {
      return;
    }

    portENTER_CRITICAL(&receiveMux);
    receivedLeft = constrain(packet.left, -255, 255);
    receivedRight = constrain(packet.right, -255, 255);
    receivedMode = static_cast<RobotMode>(packet.mode);
    receivedModeCommandId = packet.modeCommandId;
    receivedAt = millis();
    portEXIT_CRITICAL(&receiveMux);
    return;
  }

  if (header.type == PACKET_TELEMETRY &&
      IS_COORDINATOR &&
      header.source != ROBOT_ID &&
      length == static_cast<int>(sizeof(TelemetryPacket)))
  {
    TelemetryPacket packet;
    memcpy(&packet, data, sizeof(packet));

    portENTER_CRITICAL(&receiveMux);
    teamObservations[packet.source].distanceCm = packet.distanceCm;
    teamObservations[packet.source].blocked = packet.blocked != 0;
    teamObservations[packet.source].receivedAt = millis();
    portEXIT_CRITICAL(&receiveMux);
    return;
  }

  if (header.type == PACKET_SCAN &&
      IS_COORDINATOR &&
      header.source == ROLE_MAPPER &&
      length == static_cast<int>(sizeof(ScanPacket)))
  {
    portENTER_CRITICAL(&receiveMux);
    memcpy(&teamScan, data, sizeof(teamScan));
    teamScanReceivedAt = millis();
    portEXIT_CRITICAL(&receiveMux);
  }
}

String roleToString(RobotRole role)
{
  switch (role)
  {
    case ROLE_EXPLORER:
      return "Explorer";
    case ROLE_MAPPER:
      return "Mapper";
    default:
      return "Coordinator";
  }
}

float readBatteryVoltage()
{
  int raw = analogRead(BATTERY_PIN);

  if (raw <= 0)
  {
    return 0.0f;
  }

  float adcVoltage = (raw / 4095.0f) * 3.3f;
  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

long readObstacleDistanceCm()
{
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000);

  if (durationUs <= 0)
  {
    return ULTRASONIC_NO_ECHO_CM;
  }

  ultrasonicHasEchoed = true;
  return (durationUs * 0.0343f) / 2.0f;
}

RobotMode parseMode(const String &modeText)
{
  if (modeText == "avoid")
  {
    return MODE_AVOID;
  }

  if (modeText == "idle")
  {
    return MODE_IDLE;
  }

  if (modeText == "line")
  {
    return MODE_LINEFOLLOW;
  }

  if (modeText == "explore")
  {
    return MODE_EXPLORE;
  }

  if (modeText == "map")
  {
    return MODE_MAP;
  }

  if (modeText == "coordinate")
  {
    return MODE_COORDINATE;
  }

  return MODE_MANUAL;
}

String modeToString(RobotMode mode)
{
  switch (mode)
  {
    case MODE_AVOID:
      return "avoid";
    case MODE_IDLE:
      return "idle";
    case MODE_LINEFOLLOW:
      return "line";
    case MODE_EXPLORE:
      return "explore";
    case MODE_MAP:
      return "map";
    case MODE_COORDINATE:
      return "coordinate";
    case MODE_TURNAROUND:
      return "turnaround";
    default:
      return "manual";
  }
}

void followLine();

// Car 3 in Explore: keep whatever is directly ahead (Car 1) about
// FOLLOW_TARGET_CM away. Too close backs off; nothing within FOLLOW_LOST_CM
// (or no echo) means Car 1 is out of sight, so stop rather than drive blind.
int followSpeed(int distanceCm)
{
  if (distanceCm >= ULTRASONIC_NO_ECHO_CM || distanceCm > FOLLOW_LOST_CM)
  {
    return 0;
  }

  if (distanceCm < FOLLOW_TOO_CLOSE_CM)
  {
    return -FOLLOW_BACKOFF_SPEED;
  }

  if (distanceCm <= FOLLOW_TARGET_CM)
  {
    return 0;
  }

  return map(distanceCm, FOLLOW_TARGET_CM, FOLLOW_LOST_CM, FOLLOW_MIN_SPEED, FOLLOW_MAX_SPEED);
}

// A role mode gives each car a safe default task without pretending that a
// single front distance sensor can produce a full SLAM map or localization.
// Manual, Avoid and Line modes remain available for calibration and testing.
void applyRoleMode(RobotMode mode, int requestedLeft, int requestedRight)
{
  if (mode == MODE_EXPLORE)
  {
    if (ROBOT_ROLE == ROLE_EXPLORER)
    {
      targetLeft = EXPLORE_SPEED;
      targetRight = EXPLORE_SPEED;
    }
    else if (ROBOT_ROLE == ROLE_COORDINATOR)
    {
      int speed = followSpeed(lastObstacleDistanceCm);
      targetLeft = speed;
      targetRight = speed;
    }
    else
    {
      targetLeft = 0;
      targetRight = 0;
    }
    return;
  }

  if (mode == MODE_MAP || mode == MODE_COORDINATE || mode == MODE_IDLE ||
      mode == MODE_TURNAROUND)
  {
    targetLeft = 0;
    targetRight = 0;
    return;
  }

  if (mode == MODE_LINEFOLLOW)
  {
    followLine();
    return;
  }

  targetLeft = requestedLeft;
  targetRight = requestedRight;
}

String avoidancePhaseToString()
{
  switch (avoidancePhase)
  {
    case AVOIDANCE_STOPPING:
      return "Stopping for obstacle";
    case AVOIDANCE_REVERSING:
      return "Reversing from obstacle";
    case AVOIDANCE_SCAN_LEFT:
    case AVOIDANCE_READ_LEFT:
      return "Scanning left path";
    case AVOIDANCE_SCAN_RIGHT:
    case AVOIDANCE_READ_RIGHT:
      return "Scanning right path";
    case AVOIDANCE_CHOOSE_PATH:
      return "Choosing clear path";
    case AVOIDANCE_LOCKED:
      return "Blocked - no safe direction";
    default:
      return "Ready";
  }
}

void followLine()
{
  bool leftLine = (digitalRead(LINE_LEFT_PIN) == LOW);
  bool rightLine = (digitalRead(LINE_RIGHT_PIN) == LOW);

  int leftSpeed = 165;
  int rightSpeed = 165;

  if (leftLine && !rightLine)
  {
    leftSpeed = 90;
    rightSpeed = 190;
    lastLineTurn = -1;
  }
  else if (!leftLine && rightLine)
  {
    leftSpeed = 190;
    rightSpeed = 90;
    lastLineTurn = 1;
  }
  else if (leftLine && rightLine)
  {
    leftSpeed = 165;
    rightSpeed = 165;
  }
  else
  {
    if (lastLineTurn == -1)
    {
      leftSpeed = 115;
      rightSpeed = 175;
    }
    else if (lastLineTurn == 1)
    {
      leftSpeed = 175;
      rightSpeed = 115;
    }
    else
    {
      leftSpeed = 0;
      rightSpeed = 0;
    }
  }

  targetLeft = leftSpeed;
  targetRight = rightSpeed;
}

void applyObstacleAvoidance(uint32_t now, int requestedLeft, int requestedRight)
{
  bool forwardRequested = (requestedLeft + requestedRight) > 0;
  // No echo means nothing within range, unless the sensor has never answered
  // since boot (likely unplugged) - then it must still count as blocked.
  int distanceCm = lastObstacleDistanceCm;
  if (distanceCm >= ULTRASONIC_NO_ECHO_CM)
  {
    distanceCm = ultrasonicHasEchoed ? ULTRASONIC_MAX_RANGE_CM : 0;
  }
  bool obstacleClose = distanceCm <= OBSTACLE_DISTANCE_CM;
  bool pathClear = distanceCm >= OBSTACLE_CLEAR_DISTANCE_CM;

  if (!forwardRequested)
  {
    avoidancePhase = AVOIDANCE_READY;
    targetLeft = requestedLeft;
    targetRight = requestedRight;
    return;
  }

  if (avoidancePhase == AVOIDANCE_LOCKED)
  {
    targetLeft = 0;
    targetRight = 0;
    return;
  }

  if (avoidancePhase == AVOIDANCE_READY && obstacleClose)
  {
    avoidancePhase = AVOIDANCE_STOPPING;
    avoidancePhaseStartedAt = now;
    Serial.printf("Explorer obstacle/no echo at %d cm; stopping to scan\n", lastObstacleDistanceCm);
  }

  switch (avoidancePhase)
  {
    case AVOIDANCE_STOPPING:
      targetLeft = 0;
      targetRight = 0;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_STOP_MS)
      {
        avoidancePhase = AVOIDANCE_REVERSING;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_REVERSING:
      targetLeft = -AVOIDANCE_REVERSE_SPEED;
      targetRight = -AVOIDANCE_REVERSE_SPEED;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_REVERSE_MS)
      {
        avoidancePhase = AVOIDANCE_SCAN_LEFT;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_SCAN_LEFT:
      // Rotate left, then let the sensor settle before recording that path.
      targetLeft = -AVOIDANCE_TURN_SPEED;
      targetRight = AVOIDANCE_TURN_SPEED;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_TURN_90_MS)
      {
        avoidancePhase = AVOIDANCE_READ_LEFT;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_READ_LEFT:
      targetLeft = 0;
      targetRight = 0;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_SETTLE_MS)
      {
        leftScanDistanceCm = pathClear ? distanceCm : 0;
        avoidancePhase = AVOIDANCE_SCAN_RIGHT;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_SCAN_RIGHT:
      // From left-facing, turn through the original heading to right-facing.
      targetLeft = AVOIDANCE_TURN_SPEED;
      targetRight = -AVOIDANCE_TURN_SPEED;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_TURN_90_MS * 2)
      {
        avoidancePhase = AVOIDANCE_READ_RIGHT;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_READ_RIGHT:
      targetLeft = 0;
      targetRight = 0;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_SETTLE_MS)
      {
        rightScanDistanceCm = pathClear ? distanceCm : 0;
        // Same rule as AVOIDANCE_CHOOSE_PATH; 0 means "not clear".
        Serial.printf("Scan result: left %d cm, right %d cm -> %s\n",
                      leftScanDistanceCm, rightScanDistanceCm,
                      (leftScanDistanceCm == 0 && rightScanDistanceCm == 0) ? "BLOCKED, stopping"
                      : rightScanDistanceCm >= leftScanDistanceCm          ? "taking RIGHT"
                                                                           : "taking LEFT");
        avoidancePhase = AVOIDANCE_CHOOSE_PATH;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_CHOOSE_PATH:
      if (leftScanDistanceCm == 0 && rightScanDistanceCm == 0)
      {
        avoidancePhase = AVOIDANCE_LOCKED;
        targetLeft = 0;
        targetRight = 0;
      }
      else if (rightScanDistanceCm >= leftScanDistanceCm)
      {
        // Already facing the clearer right path.
        avoidancePhase = AVOIDANCE_READY;
        targetLeft = 0;
        targetRight = 0;
      }
      else
      {
        // Return from right-facing to the clearer left path.
        targetLeft = -AVOIDANCE_TURN_SPEED;
        targetRight = AVOIDANCE_TURN_SPEED;
        if (now - avoidancePhaseStartedAt >= AVOIDANCE_TURN_90_MS * 2)
        {
          avoidancePhase = AVOIDANCE_READY;
          targetLeft = 0;
          targetRight = 0;
        }
      }
      break;

    case AVOIDANCE_LOCKED:
      targetLeft = 0;
      targetRight = 0;
      break;

    case AVOIDANCE_READY:
    default:
      targetLeft = requestedLeft;
      targetRight = requestedRight;
      break;
  }
}

// Car 2 in Map mode: measure, turn 90 degrees right, repeat four times, ending
// at the starting heading. Runs once per Map button press, then holds.
void updateObserverScan(uint32_t now, RobotMode mode, uint8_t commandId)
{
  static uint8_t scannedForCommandId = 0;

  if (mode != MODE_MAP)
  {
    observerPhase = OBSERVER_IDLE;
    return;
  }

  // Pressing Map again (new command id) starts a fresh scan.
  if (observerPhase == OBSERVER_IDLE || commandId != scannedForCommandId)
  {
    scannedForCommandId = commandId;
    ++observerScanId;
    observerReadings = 0;
    for (int i = 0; i < OBSERVER_SCAN_STEPS; ++i)
    {
      observerDistanceCm[i] = 0;
    }
    observerPhase = OBSERVER_SETTLE;
    observerPhaseStartedAt = now;
  }

  switch (observerPhase)
  {
    case OBSERVER_SETTLE:
      targetLeft = 0;
      targetRight = 0;
      if (now - observerPhaseStartedAt >= AVOIDANCE_SETTLE_MS)
      {
        // As for Car 1: no echo is open space only if the sensor ever answered.
        observerDistanceCm[observerReadings] =
            (lastObstacleDistanceCm >= ULTRASONIC_NO_ECHO_CM && !ultrasonicHasEchoed)
                ? 0
                : constrain(lastObstacleDistanceCm, 0, ULTRASONIC_NO_ECHO_CM);
        ++observerReadings;
        observerPhase = OBSERVER_TURN;
        observerPhaseStartedAt = now;
      }
      break;

    case OBSERVER_TURN:
      targetLeft = AVOIDANCE_TURN_SPEED;
      targetRight = -AVOIDANCE_TURN_SPEED;
      // Keep turning through this loop; the next phase stops the motors.
      // Stopping here would cut the turn one loop short.
      if (now - observerPhaseStartedAt >= AVOIDANCE_TURN_90_MS)
      {
        observerPhase = observerReadings < OBSERVER_SCAN_STEPS ? OBSERVER_SETTLE : OBSERVER_DONE;
        observerPhaseStartedAt = now;
        if (observerPhase == OBSERVER_DONE)
        {
          Serial.printf("Map scan #%d: front %d, right %d, back %d, left %d cm (999 = open)\n",
                        observerScanId, observerDistanceCm[0], observerDistanceCm[1],
                        observerDistanceCm[2], observerDistanceCm[3]);
        }
      }
      break;

    default:
      targetLeft = 0;
      targetRight = 0;
      break;
  }
}

bool observerScanTurning()
{
  return observerPhase == OBSERVER_TURN;
}

// Car 1 in Turnaround mode: one 180-degree turn per entry, then hold.
void updateTurnaround(uint32_t now, RobotMode mode)
{
  if (mode != MODE_TURNAROUND)
  {
    turnaroundPhase = TURNAROUND_NONE;
    return;
  }

  if (turnaroundPhase == TURNAROUND_NONE)
  {
    turnaroundPhase = TURNAROUND_TURNING;
    turnaroundStartedAt = now;
  }

  // The turn starts on the entry loop, so stop on the first loop at or past
  // the turn time; the motors then run for exactly the elapsed time.
  if (turnaroundPhase == TURNAROUND_TURNING &&
      now - turnaroundStartedAt >= AVOIDANCE_TURN_90_MS * 2)
  {
    turnaroundPhase = TURNAROUND_DONE;
  }

  if (turnaroundPhase == TURNAROUND_TURNING)
  {
    targetLeft = AVOIDANCE_TURN_SPEED;
    targetRight = -AVOIDANCE_TURN_SPEED;
    return;
  }

  targetLeft = 0;
  targetRight = 0;
}

// Car 3: a mode button press starts coordination from a clean state.
void resetCoordination(uint32_t now)
{
  explorerRecoveries = 0;
  explorerNeedsOperator = false;
  coordinatorPhaseStartedAt = now;
}

// Car 3: when Car 1 reports blocked (front, left and right all closed) during
// Explore, send it back the way it came - the one direction known to be clear -
// and resume. Give up after MAX_EXPLORER_RECOVERIES without real progress.
void coordinateExplorer(uint32_t now, bool explorerBlocked, uint32_t explorerReportedAt)
{
  if (currentMode == MODE_EXPLORE)
  {
    // Ignore a report sent before Explore resumed; it describes the old spot.
    bool newReport = static_cast<int32_t>(explorerReportedAt - coordinatorPhaseStartedAt) > 0;

    if (explorerBlocked && newReport)
    {
      if (explorerRecoveries < MAX_EXPLORER_RECOVERIES)
      {
        ++explorerRecoveries;
        currentMode = MODE_TURNAROUND;
        Serial.printf("Explorer blocked; turning it back (%d/%d)\n",
                      explorerRecoveries, MAX_EXPLORER_RECOVERIES);
      }
      else
      {
        currentMode = MODE_COORDINATE;
        explorerNeedsOperator = true;
        Serial.println("Explorer still blocked; team paused for the operator.");
      }
      coordinatorPhaseStartedAt = now;
    }
    else if (now - coordinatorPhaseStartedAt >= EXPLORER_PROGRESS_MS)
    {
      // Explored freely for a while: earlier dead ends no longer count.
      explorerRecoveries = 0;
    }
  }
  else if (currentMode == MODE_TURNAROUND &&
           now - coordinatorPhaseStartedAt >= TURNAROUND_RESUME_MS)
  {
    currentMode = MODE_EXPLORE;
    coordinatorPhaseStartedAt = now;
  }
}

bool avoidanceNeedsImmediateDrive()
{
  return avoidancePhase != AVOIDANCE_READY;
}

// Arduino ESP32 Core 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void onReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int length)
{
  processPacket(data, length);
}

#else

// Arduino ESP32 Core 2.x
void onReceive(
    const uint8_t *mac,
    const uint8_t *data,
    int length)
{
  processPacket(data, length);
}

#endif

// Web remote controller
const char PAGE[] PROGMEM = R"SWARMHTML(
<!DOCTYPE html>
<html lang="en">

<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SWARM-X Remote</title>
<style>
  :root {
    --bg: #0d1727;
    --panel: #1b2b3e;
    --panel-2: #23364d;
    --line: #35506d;
    --text: #edf5ff;
    --muted: #a8bdd9;
    --ok: #7fe0b3;
    --warn: #f3d97a;
    --danger: #ff8a8a;
    --accent: #2bb2d0;
    --accent-2: #4f9cff;
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    background: linear-gradient(180deg, #0d1727 0%, #111b2d 100%);
    color: var(--text);
    font-family: Arial, sans-serif;
    min-height: 100vh;
    display: flex;
    justify-content: center;
    align-items: center;
  }

  .app {
    width: min(560px, 100%);
    padding: 20px 18px 30px;
  }

  .header {
    text-align: center;
    margin-bottom: 14px;
  }

  h1 {
    margin: 0;
    font-size: clamp(1.8rem, 5vw, 2.6rem);
  }

  .subtitle {
    margin: 8px 0 0;
    color: var(--muted);
    font-size: 0.95rem;
  }

  .panel {
    background: rgba(21, 35, 50, 0.9);
    border: 1px solid var(--line);
    border-radius: 16px;
    padding: 16px;
    margin-bottom: 16px;
    box-shadow: 0 8px 20px rgba(0,0,0,0.18);
  }

  .status {
    min-height: 58px;
    display: flex;
    align-items: center;
    justify-content: center;
    text-align: center;
    font-weight: bold;
    border-radius: 12px;
    background: #1a2d41;
    border: 1px solid var(--line);
    color: var(--ok);
    padding: 12px;
  }

  .status.warning { color: var(--warn); }
  .status.danger { color: var(--danger); }

  .stats {
    display: grid;
    grid-template-columns: repeat(6, minmax(0, 1fr));
    gap: 10px;
    margin-top: 12px;
  }

  .stat { grid-column: span 2; }
  .stat.wide { grid-column: span 3; }
  .stat-value.offline { color: var(--muted); }
  .stat-value.blocked { color: var(--danger); }
  .stat-value.open { color: var(--ok); }

  .scan {
    margin-top: 12px;
    background: var(--panel-2);
    border: 1px solid var(--line);
    border-radius: 12px;
    padding: 10px 8px;
    text-align: center;
  }

  .scan-grid {
    display: grid;
    grid-template-columns: repeat(4, minmax(0, 1fr));
    gap: 6px;
  }

  .stat {
    background: var(--panel-2);
    border: 1px solid var(--line);
    border-radius: 12px;
    padding: 10px 8px;
    text-align: center;
  }

  .stat-label {
    display: block;
    font-size: 0.72rem;
    color: var(--muted);
    margin-bottom: 6px;
  }

  .stat-value {
    font-size: 1.05rem;
    font-weight: bold;
  }

  .mode-bar {
    display: flex;
    gap: 8px;
    margin-bottom: 16px;
  }

  .mode-btn, .btn {
    appearance: none;
    border: 1px solid var(--line);
    background: var(--panel-2);
    color: var(--text);
    border-radius: 12px;
    padding: 12px 14px;
    font-weight: bold;
    font-size: 0.96rem;
    cursor: pointer;
    touch-action: manipulation;
    user-select: none;
  }

  .mode-btn.active {
    background: linear-gradient(180deg, var(--accent), var(--accent-2));
    border-color: transparent;
  }

  .mode-btn { flex: 1; }

  .joystick-wrap {
    position: relative;
    width: 100%;
    height: 220px;
    border-radius: 22px;
    background: rgba(16, 27, 39, 0.85);
    border: 2px solid var(--line);
    touch-action: none;
    overflow: hidden;
  }

  .joystick-ring {
    position: absolute;
    left: 50%;
    top: 50%;
    width: 160px;
    height: 160px;
    margin-left: -80px;
    margin-top: -80px;
    border-radius: 50%;
    border: 2px solid rgba(255,255,255,0.12);
    background: radial-gradient(circle, rgba(17,89,114,0.3), rgba(15,22,31,0.4));
  }

  .joystick-stick {
    position: absolute;
    left: 50%;
    top: 50%;
    width: 74px;
    height: 74px;
    margin-left: -37px;
    margin-top: -37px;
    border-radius: 50%;
    background: linear-gradient(180deg, #46c5ff, #1a8ec2);
    border: 2px solid rgba(255,255,255,0.25);
    box-shadow: 0 8px 20px rgba(16, 91, 128, 0.45);
    pointer-events: none;
  }

  .controls-row {
    display: flex;
    gap: 10px;
    margin-top: 14px;
  }

  .btn {
    flex: 1;
    min-height: 52px;
  }

  .stop-btn {
    background: linear-gradient(180deg, #9a3a4b, #7f2439);
  }

  .speed-box {
    margin-top: 16px;
  }

  .speed-label {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 8px;
    color: var(--muted);
    font-size: 0.95rem;
  }

  input[type=range] {
    width: 100%;
    accent-color: var(--accent);
  }

  .info {
    font-size: 0.82rem;
    color: var(--muted);
    margin-top: 12px;
    line-height: 1.7;
  }
</style>
</head>
<body>
  <div class="app">
    <div class="header">
      <h1>SWARM-X</h1>
      <p class="subtitle">Three-Robot Cooperative Mission Control</p>
    </div>

    <div class="panel">
      <div id="status" class="status">Ready - waiting for command</div>

      <div class="stats">
        <div class="stat">
          <span class="stat-label">Battery</span>
          <span id="batteryValue" class="stat-value">N/A</span>
        </div>
        <div class="stat">
          <span class="stat-label">Mode</span>
          <span id="modeValue" class="stat-value">Manual</span>
        </div>
        <div class="stat">
          <span class="stat-label">Car 3 Coordinator</span>
          <span id="obstacleValue" class="stat-value">-- cm</span>
        </div>
        <div class="stat wide">
          <span class="stat-label">Car 1 Explorer</span>
          <span id="car1Value" class="stat-value">--</span>
        </div>
        <div class="stat wide">
          <span class="stat-label">Car 2 Observer</span>
          <span id="car2Value" class="stat-value">--</span>
        </div>
      </div>

      <div class="scan">
        <span class="stat-label">Car 2 scan, relative to its heading - <b id="scanState">press Map to scan</b></span>
        <div class="scan-grid">
          <div><span class="stat-label">Front</span><span class="scan-cell stat-value">--</span></div>
          <div><span class="stat-label">Right</span><span class="scan-cell stat-value">--</span></div>
          <div><span class="stat-label">Back</span><span class="scan-cell stat-value">--</span></div>
          <div><span class="stat-label">Left</span><span class="scan-cell stat-value">--</span></div>
        </div>
      </div>
    </div>

    <div class="mode-bar">
      <button class="mode-btn active" data-mode="manual">Manual</button>
      <button class="mode-btn" data-mode="avoid">Avoid</button>
      <button class="mode-btn" data-mode="line">Line</button>
      <button class="mode-btn" data-mode="explore">Explore</button>
      <button class="mode-btn" data-mode="map">Map</button>
      <button class="mode-btn" data-mode="coordinate">Coordinate</button>
      <button class="mode-btn" data-mode="idle">Idle</button>
    </div>

    <div class="panel">
      <div id="joystick" class="joystick-wrap" aria-label="Drive joystick">
        <div class="joystick-ring"></div>
        <div id="stick" class="joystick-stick"></div>
      </div>

      <div class="controls-row">
        <button id="stop" class="btn stop-btn">STOP</button>
      </div>

      <div class="speed-box">
        <div class="speed-label">
          <span>Motor Speed</span>
          <b id="speedValue">200</b>/255
        </div>
        <input id="speed" type="range" min="100" max="255" step="5" value="200" />
      </div>

      <div class="info">
        Drag the joystick or use WASD / Arrow keys.<br>
        <b>Explore:</b> Car 1 drives; at 35 cm it reverses, scans left and right, and takes a path with 60 cm clear.
        If none is clear, Car 3 turns it back (up to 2 times) before pausing the team. Car 3 follows Car 1 at about 50 cm.<br>
        <b>Map:</b> Car 2 measures front, right, back and left; press Map again to rescan.<br>
        All missions stop if this page is closed for 2 seconds.<br>
        Battery cutoff stays disabled until a voltage divider is installed and enabled.
      </div>
    </div>
  </div>

  <script>
    const speed = document.getElementById('speed');
    const speedValue = document.getElementById('speedValue');
    const status = document.getElementById('status');
    const batteryValue = document.getElementById('batteryValue');
    const obstacleValue = document.getElementById('obstacleValue');
    const modeValue = document.getElementById('modeValue');
    const car1Value = document.getElementById('car1Value');
    const car2Value = document.getElementById('car2Value');
    const scanState = document.getElementById('scanState');
    const scanCells = [...document.querySelectorAll('.scan-cell')];
    const joystickEl = document.getElementById('joystick');
    const stickEl = document.getElementById('stick');
    const modeButtons = [...document.querySelectorAll('.mode-btn')];

    let activeMode = 'manual';
    let busy = false;
    let pendingDrive = null;
    let driveError = false;
    let driveFailures = 0;
    let statusBusy = false;
    let modeChangedAt = 0;
    let stickState = { active: false, x: 0, y: 0 };
    let keys = new Set();

    const directions = {
      ArrowUp: 'up',
      KeyW: 'up',
      ArrowDown: 'down',
      KeyS: 'down',
      ArrowLeft: 'left',
      KeyA: 'left',
      ArrowRight: 'right',
      KeyD: 'right'
    };

    function showMode(mode) {
      activeMode = mode;
      modeButtons.forEach(btn => btn.classList.toggle('active', btn.dataset.mode === mode));
      modeValue.textContent = mode === 'line' ? 'Line' : mode.charAt(0).toUpperCase() + mode.slice(1);
    }

    // 999 cm means no echo: nothing within the sensor's ~4 m range.
    function formatDistance(cm) {
      return cm >= 999 ? 'open' : cm + ' cm';
    }

    function showTeammate(el, car) {
      if (!car) return;
      let text = 'offline';
      let state = 'offline';
      if (car.online) {
        text = car.blocked ? 'blocked' : formatDistance(car.distance);
        state = car.blocked ? 'blocked' : '';
      }
      el.textContent = text;
      el.className = 'stat-value ' + state;
    }

    // A direction is open when it is at least as far as Car 1 needs to pass.
    function showScan(stateEl, cells, scan, clearCm) {
      if (!scan) return;
      stateEl.textContent = scan.readings < cells.length
        ? 'scanning ' + scan.readings + '/' + cells.length
        : 'scan #' + scan.id;
      cells.forEach((cell, i) => {
        if (i >= scan.readings) {
          cell.textContent = '...';
          cell.className = 'scan-cell stat-value offline';
          return;
        }
        const cm = scan.cm[i];
        cell.textContent = formatDistance(cm);
        cell.className = 'scan-cell stat-value ' + (cm >= clearCm ? 'open' : 'blocked');
      });
    }

    function setMode(mode) {
      showMode(mode);
      modeChangedAt = Date.now();
      fetch('/mode?m=' + mode, { cache: 'no-store' });
      if (mode === 'idle') {
        updateDrive(0, 0);
      }
    }

    function wheelSpeedsFromAxes(f, t) {
      const s = Number(speed.value);
      let left = 0;
      let right = 0;

      if (f !== 0) {
        left = f * s;
        right = f * s;
        if (t > 0) left = f * Math.round(s * 0.45);
        if (t < 0) right = f * Math.round(s * 0.45);
      } else if (t !== 0) {
        left = -t * s;
        right = t * s;
      }

      return [left, right];
    }

    function currentAxes() {
      if (stickState.active) {
        const y = -stickState.y;
        const x = stickState.x;
        return { f: Math.abs(y) > 0.15 ? y : 0, t: Math.abs(x) > 0.15 ? x : 0 };
      }

      return {
        f: Number(keys.has('up')) - Number(keys.has('down')),
        t: Number(keys.has('right')) - Number(keys.has('left'))
      };
    }

    function updateDrive(left, right) {
      pendingDrive = { left, right };
      sendPendingDrive();
    }

    function sendPendingDrive() {
      if (busy || !pendingDrive) return;
      busy = true;
      const command = pendingDrive;
      pendingDrive = null;
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 400);

      fetch('/drive?l=' + Math.round(command.left) + '&r=' + Math.round(command.right), {
        cache: 'no-store',
        signal: controller.signal
      })
      .then(resp => resp.text())
      .then(text => {
        if (text && text.trim() === 'OK') {
          driveFailures = 0;
          driveError = false;
          status.textContent = 'Connected - motors active';
          status.className = 'status';
        } else {
          driveError = true;
          status.textContent = 'Command rejected';
          status.className = 'status danger';
        }
      })
      .catch(() => {
        // One slow reply is harmless: the car keeps the last command for
        // 0.8 s. Only two failures in a row mean the link is really down.
        driveFailures++;
        if (driveFailures >= 2) {
          driveError = true;
          status.textContent = 'Connection problem - motors should stop';
          status.className = 'status danger';
        }
      })
      .finally(() => {
        clearTimeout(timeout);
        busy = false;
        sendPendingDrive();
      });
    }

    function sendCurrentAxes() {
      const { f, t } = currentAxes();
      const [left, right] = wheelSpeedsFromAxes(f, t);
      updateDrive(left, right);
    }

    function stopDriving() {
      stickState.active = false;
      stickState.x = 0;
      stickState.y = 0;
      updateStick();
      keys.clear();
      if (activeMode !== 'idle') {
        updateDrive(0, 0);
      }
    }

    function updateStick() {
      const max = 52;
      const moveX = Math.max(-1, Math.min(1, stickState.x));
      const moveY = Math.max(-1, Math.min(1, stickState.y));
      const dx = moveX * max;
      const dy = moveY * max;
      stickEl.style.transform = 'translate(' + dx + 'px, ' + dy + 'px)';
    }

    function setJoystickFromPointer(clientX, clientY) {
      const rect = joystickEl.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      const dx = clientX - cx;
      const dy = clientY - cy;
      const distance = Math.min(Math.hypot(dx, dy), 68);
      const angle = Math.atan2(dy, dx);
      const nx = Math.cos(angle) * distance / 68;
      const ny = Math.sin(angle) * distance / 68;
      stickState.active = true;
      stickState.x = nx;
      // Screen Y grows downward; currentAxes() turns "up" into forward.
      stickState.y = ny;
      updateStick();
      sendCurrentAxes();
    }

    joystickEl.addEventListener('pointerdown', e => {
      e.preventDefault();
      joystickEl.setPointerCapture(e.pointerId);
      setJoystickFromPointer(e.clientX, e.clientY);
    });

    joystickEl.addEventListener('pointermove', e => {
      if (e.pressure === 0 && !stickState.active) return;
      if (e.buttons === 1 || stickState.active) {
        setJoystickFromPointer(e.clientX, e.clientY);
      }
    });

    joystickEl.addEventListener('pointerup', stopDriving);
    joystickEl.addEventListener('pointercancel', stopDriving);
    joystickEl.addEventListener('lostpointercapture', stopDriving);

    modeButtons.forEach(btn => {
      btn.addEventListener('click', () => setMode(btn.dataset.mode));
    });

    document.getElementById('stop').addEventListener('click', stopDriving);

    window.addEventListener('keydown', e => {
      if (directions[e.code]) {
        e.preventDefault();
        keys.add(directions[e.code]);
        sendCurrentAxes();
      }
      if (e.code === 'Space') {
        e.preventDefault();
        stopDriving();
      }
    });

    window.addEventListener('keyup', e => {
      if (directions[e.code]) {
        e.preventDefault();
        keys.delete(directions[e.code]);
        sendCurrentAxes();
      }
    });

    window.addEventListener('blur', stopDriving);
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) stopDriving();
    });

    speed.addEventListener('input', () => {
      speedValue.textContent = speed.value;
      sendCurrentAxes();
    });

    // One status request at a time: piled-up requests use up the browser's
    // connections to the car and delay the drive commands behind them.
    function pollStatus() {
      if (statusBusy) return;
      statusBusy = true;
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 1000);

      fetch('/status', { cache: 'no-store', signal: controller.signal })
        .then(r => r.json())
        .then(data => {
          if (!data.batteryEnabled) {
            batteryValue.textContent = 'N/A';
          } else if (typeof data.battery === 'number') {
            batteryValue.textContent = Number(data.battery).toFixed(1) + 'V';
          }
          if (typeof data.distance === 'number') obstacleValue.textContent = formatDistance(data.distance);
          // Follow mode changes made by the car itself (e.g. auto-pause), but
          // not a stale reply that raced a button press.
          if (data.mode && data.mode !== activeMode && Date.now() - modeChangedAt > 1000) showMode(data.mode);
          showTeammate(car1Value, data.c1);
          showTeammate(car2Value, data.c2);
          showScan(scanState, scanCells, data.c2scan, data.clearCm);
          if (data.status && (data.warn || !driveError)) {
            status.textContent = data.status;
            status.className = 'status ' + (data.warn ? 'warning' : '');
          }
        })
        .catch(() => {})
        .finally(() => {
          clearTimeout(timeout);
          statusBusy = false;
        });
    }

    setInterval(() => {
      pollStatus();
      if (!document.hidden) {
        sendCurrentAxes();
      }
    }, 250);

    speedValue.textContent = speed.value;
    setMode('manual');
    updateStick();
  </script>
</body>
</html>
)SWARMHTML";

// One teammate's latest telemetry for the dashboard's /status reply.
String teamObservationJson(const TeamObservation &car, uint32_t now)
{
  bool online = car.receivedAt != 0 &&
                now - car.receivedAt <= TEAM_TELEMETRY_TIMEOUT_MS;
  return String("{\"online\":") + (online ? "true" : "false") +
         ",\"distance\":" + String(car.distanceCm) +
         ",\"blocked\":" + (car.blocked ? "true" : "false") + "}";
}

// Car 2's latest scan for /status, or null if none has arrived yet.
String scanJson(const ScanPacket &scan, uint32_t receivedAt)
{
  if (receivedAt == 0)
  {
    return "null";
  }

  String json = String("{\"id\":") + String(scan.scanId) +
                ",\"readings\":" + String(scan.readings) + ",\"cm\":[";
  for (int i = 0; i < OBSERVER_SCAN_STEPS; ++i)
  {
    if (i > 0)
    {
      json += ",";
    }
    json += String(scan.distanceCm[i]);
  }
  return json + "]}";
}

// ---- Car 3 web server ----

// Copies one query parameter (e.g. "l" from /drive?l=100) into value.
bool queryValue(httpd_req_t *req, const char *key, char *value, size_t size)
{
  char query[64];
  return httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
         httpd_query_key_value(query, key, value, size) == ESP_OK;
}

esp_err_t sendResponse(httpd_req_t *req, const char *status, const char *type,
                       const char *body, ssize_t length = HTTPD_RESP_USE_STRLEN)
{
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, type);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, body, length);
}

esp_err_t handleRoot(httpd_req_t *req)
{
  return sendResponse(req, "200 OK", "text/html", PAGE);
}

// Motor command from the page. Mode changes only through /mode, so a periodic
// drive command cannot undo the coordinator's automatic pause.
esp_err_t handleDrive(httpd_req_t *req)
{
  char left[8];
  char right[8];
  if (!queryValue(req, "l", left, sizeof(left)) ||
      !queryValue(req, "r", right, sizeof(right)))
  {
    return sendResponse(req, "400 Bad Request", "text/plain", "Missing command");
  }

  int16_t leftSpeed = constrain(atoi(left), -255, 255);
  int16_t rightSpeed = constrain(atoi(right), -255, 255);

  portENTER_CRITICAL(&webMux);
  webLeft = leftSpeed;
  webRight = rightSpeed;
  webCommandAt = millis();
  portEXIT_CRITICAL(&webMux);

  return sendResponse(req, "200 OK", "text/plain", "OK");
}

esp_err_t handleMode(httpd_req_t *req)
{
  char modeText[16];
  if (!queryValue(req, "m", modeText, sizeof(modeText)))
  {
    return sendResponse(req, "400 Bad Request", "text/plain", "Missing mode");
  }

  RobotMode mode = parseMode(String(modeText));

  portENTER_CRITICAL(&webMux);
  pendingMode = mode;
  webCommandAt = millis();
  portEXIT_CRITICAL(&webMux);

  return sendResponse(req, "200 OK", "text/plain", modeToString(mode).c_str());
}

esp_err_t handleStatus(httpd_req_t *req)
{
  // Copy under the lock; build Strings only after releasing it.
  TeamObservation explorer;
  TeamObservation mapper;
  ScanPacket scan;
  uint32_t scanReceivedAt;
  portENTER_CRITICAL(&receiveMux);
  uint32_t now = millis();
  explorer = teamObservations[ROLE_EXPLORER];
  mapper = teamObservations[ROLE_MAPPER];
  scan = teamScan;
  scanReceivedAt = teamScanReceivedAt;
  portEXIT_CRITICAL(&receiveMux);

  String payload = "{\"batteryEnabled\":" +
                   String(BATTERY_MONITOR_ENABLED ? "true" : "false") +
                   ",\"battery\":" +
                   (BATTERY_MONITOR_ENABLED ? String(batteryVoltage, 1) : String("null")) +
                   ",\"distance\":" + String(lastObstacleDistanceCm) +
                   ",\"mode\":\"" + modeToString(currentMode) +
                   "\",\"role\":\"Coordinator\"" +
                   ",\"c1\":" + teamObservationJson(explorer, now) +
                   ",\"c2\":" + teamObservationJson(mapper, now) +
                   ",\"c2scan\":" + scanJson(scan, scanReceivedAt) +
                   ",\"clearCm\":" + String(OBSTACLE_CLEAR_DISTANCE_CM) +
                   ",\"avoidance\":\"" + avoidancePhaseToString() + "\"";

  if (BATTERY_MONITOR_ENABLED && batteryVoltage < BATTERY_LOW_VOLTAGE)
  {
    payload += ",\"warn\":true,\"status\":\"Low battery - motors disabled\"";
  }
  else if (explorerNeedsOperator)
  {
    payload += ",\"warn\":true,\"status\":\"Car 1 still blocked after " +
               String(MAX_EXPLORER_RECOVERIES) +
               " turn-backs - team paused, choose a mode\"";
  }
  else if (currentMode == MODE_TURNAROUND)
  {
    payload += ",\"warn\":false,\"status\":\"Car 1 blocked - turning it back (" +
               String(explorerRecoveries) + "/" + String(MAX_EXPLORER_RECOVERIES) + ")\"";
  }
  else if (avoidancePhase == AVOIDANCE_LOCKED)
  {
    payload += ",\"warn\":true,\"status\":\"Blocked - no safe direction\"";
  }
  else if (avoidancePhase != AVOIDANCE_READY)
  {
    payload += ",\"warn\":false,\"status\":\"" + avoidancePhaseToString() + "\"";
  }
  else
  {
    payload += ",\"warn\":false,\"status\":\"Connected - motors active\"";
  }

  payload += "}";
  return sendResponse(req, "200 OK", "application/json", payload.c_str(), payload.length());
}

esp_err_t handleNotFound(httpd_req_t *req, httpd_err_code_t)
{
  return sendResponse(req, "404 Not Found", "text/plain", "Not found");
}

void startWebServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  // When all sockets are busy, close the least recently used one (such as an
  // idle connection the browser opened in advance) instead of refusing.
  config.lru_purge_enable = true;
  config.recv_wait_timeout = 2;
  config.send_wait_timeout = 2;
  config.stack_size = 8192; // /status builds its reply with String

  if (httpd_start(&httpServer, &config) != ESP_OK)
  {
    Serial.println("Web server failed to start; dashboard unavailable.");
    return;
  }

  const httpd_uri_t routes[] = {
      {"/", HTTP_GET, handleRoot, nullptr},
      {"/drive", HTTP_GET, handleDrive, nullptr},
      {"/mode", HTTP_GET, handleMode, nullptr},
      {"/status", HTTP_GET, handleStatus, nullptr},
  };
  for (const httpd_uri_t &route : routes)
  {
    httpd_register_uri_handler(httpServer, &route);
  }
  httpd_register_err_handler(httpServer, HTTPD_404_NOT_FOUND, handleNotFound);
}

// Home Wi-Fi diagnostics, so a failed join says why instead of guessing.
volatile uint8_t homeWifiDisconnectReason = 0;

const char *wifiReasonText(uint8_t reason)
{
  switch (reason)
  {
    case 0:
      return "no disconnect reported";
    case WIFI_REASON_AUTH_EXPIRE:
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "rejected - wrong password, or the network needs a username";
    case WIFI_REASON_NO_AP_FOUND:
      return "network not found - name wrong, out of range, or 5 GHz only";
    case WIFI_REASON_ASSOC_FAIL:
    case WIFI_REASON_ASSOC_EXPIRE:
      return "router refused the connection";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "signal lost - too far from the router";
    default:
      return "see ESP-IDF wifi_err_reason_t";
  }
}

const char *wifiAuthText(wifi_auth_mode_t auth)
{
  switch (auth)
  {
    case WIFI_AUTH_OPEN:
      return "open";
    case WIFI_AUTH_WEP:
      return "WEP";
    case WIFI_AUTH_WPA_PSK:
    case WIFI_AUTH_WPA2_PSK:
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "password (WPA/WPA2)";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "ENTERPRISE (username + password)";
    case WIFI_AUTH_WPA3_PSK:
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "password (WPA3)";
    default:
      return "other";
  }
}

// List every 2.4 GHz network Car 3 can see and say whether the configured one
// is among them. The ESP32 cannot see 5 GHz networks at all.
void reportHomeWifiScan()
{
  Serial.println("Scanning for Wi-Fi networks...");
  int found = WiFi.scanNetworks();
  bool seen = false;

  for (int i = 0; i < found; ++i)
  {
    bool match = WiFi.SSID(i) == HOME_WIFI_SSID;
    seen = seen || match;
    Serial.printf("  %s \"%s\"  channel %d  signal %d dBm  %s\n",
                  match ? ">>" : "  ", WiFi.SSID(i).c_str(), WiFi.channel(i),
                  WiFi.RSSI(i), wifiAuthText(WiFi.encryptionType(i)));
  }

  if (!seen)
  {
    Serial.printf("\"%s\" is NOT visible: check the exact name (spaces, capitals), "
                  "or it is 5 GHz only / out of range.\n",
                  HOME_WIFI_SSID);
  }

  WiFi.scanDelete();
}

// ---- Running from the car battery ----
// Starting the Wi-Fi radio draws a short ~0.5 A spike. On USB that is easy;
// through the car's battery and regulator it can dip the voltage, the
// brownout detector restarts the chip, and Wi-Fi never stays up.

const int STATUS_LED_PIN = 2; // on-board blue LED: blinks while starting, on when Wi-Fi is ready
const uint32_t POWER_SETTLE_MS = 500;
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_11dBm; // lower peak current; still covers a room
const int RESET_HISTORY_LENGTH = 5;

void setStatusLed(bool on)
{
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void blinkStatusLed()
{
  digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
}

void limitWifiPower()
{
  WiFi.setTxPower(WIFI_TX_POWER);
}

const char *resetReasonText(uint8_t reason)
{
  switch (reason)
  {
    case ESP_RST_POWERON:
      return "power-on";
    case ESP_RST_EXT:
      return "reset button";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return "watchdog";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT (power dip)";
    default:
      return "other";
  }
}

// Keeps the last few restart reasons in flash, so restarts that happened on
// battery, with no laptop attached, can be read on the next USB connection.
void reportResetHistory()
{
  Preferences prefs;
  prefs.begin("swarmx", false);
  uint32_t boots = prefs.getUInt("boots", 0) + 1;
  uint8_t history[RESET_HISTORY_LENGTH] = {};
  prefs.getBytes("resets", history, sizeof(history));
  memmove(history + 1, history, RESET_HISTORY_LENGTH - 1);
  history[0] = static_cast<uint8_t>(esp_reset_reason());
  prefs.putUInt("boots", boots);
  prefs.putBytes("resets", history, sizeof(history));
  prefs.end();

  Serial.printf("Boot #%lu. Restart reasons, newest first:", static_cast<unsigned long>(boots));
  for (int i = 0; i < RESET_HISTORY_LENGTH && history[i] != 0; ++i)
  {
    Serial.printf(" [%s]", resetReasonText(history[i]));
  }
  Serial.println();
}

// ESP32 setup
void setup()
{
  // A short voltage dip from the Wi-Fi start-up spike must not restart the car.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);
  reportResetHistory();

  pinMode(BATTERY_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);

  if (ULTRASONIC_SERIAL_TEST)
  {
    Serial.println("Ultrasonic serial test active; motors are disabled.");
    Serial.println("Distance readings start below.");
  }

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Start with motors stopped
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Car 3 is both a visible operator access point and a home-Wi-Fi station.
  // Cars 1 and 2 join the Car 3 access point.
  if (IS_COORDINATOR)
  {
    // Let the battery supply settle after power-on before the radio starts.
    delay(POWER_SETTLE_MS);
    WiFi.mode(WIFI_AP_STA);
    limitWifiPower();
    WiFi.setSleep(false);

    if (!WiFi.softAP(AP_NAME, AP_PASSWORD, RADIO_CHANNEL))
    {
      Serial.println("Could not start SWARM-X Wi-Fi; motors remain stopped.");
      while (true)
      {
        drive(0, 0);
        delay(1000);
      }
    }

    limitWifiPower();
    setStatusLed(true); // SWARM-X is up
    Serial.printf("Operator Wi-Fi: %s\n", AP_NAME);
    Serial.print("Dashboard on SWARM-X: http://");
    Serial.println(WiFi.softAPIP());

    if (strlen(HOME_WIFI_SSID) > 0 && strlen(HOME_WIFI_PASSWORD) > 0)
    {
      reportHomeWifiScan();

      WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info)
                   { homeWifiDisconnectReason = info.wifi_sta_disconnected.reason; },
                   ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

      Serial.printf("Connecting Car 3 to \"%s\"...\n", HOME_WIFI_SSID);
      WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);
      uint32_t connectStartedAt = millis();
      while (WiFi.status() != WL_CONNECTED &&
             millis() - connectStartedAt < WIFI_CONNECT_TIMEOUT_MS)
      {
        delay(250);
      }

      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.print("Car 3 home-Wi-Fi dashboard: http://");
        Serial.println(WiFi.localIP());
      }
      else
      {
        Serial.printf("Home Wi-Fi failed: status %d, reason %d (%s)\n",
                      static_cast<int>(WiFi.status()), homeWifiDisconnectReason,
                      wifiReasonText(homeWifiDisconnectReason));
        Serial.println("SWARM-X operator Wi-Fi remains active.");
      }
    }
    else
    {
      Serial.println("Home Wi-Fi credentials missing; SWARM-X operator Wi-Fi remains active.");
    }
  }
  else
  {
    delay(POWER_SETTLE_MS);
    WiFi.mode(WIFI_STA);
    limitWifiPower();
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    // Car 3 may power up later than this car, so keep trying instead of
    // giving up. Motors are already stopped above.
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.printf("Car %d joining %s...\n", ROBOT_ID, AP_NAME);
      WiFi.disconnect();
      WiFi.begin(AP_NAME, AP_PASSWORD);

      uint32_t connectStartedAt = millis();
      while (WiFi.status() != WL_CONNECTED &&
             millis() - connectStartedAt < WIFI_CONNECT_TIMEOUT_MS)
      {
        delay(250);
        blinkStatusLed(); // searching for SWARM-X
      }
    }

    setStatusLed(true);
    Serial.printf("Car %d joined %s on channel %ld\n",
                  ROBOT_ID, AP_NAME, static_cast<long>(WiFi.channel()));
  }

  // Start ESP-NOW
  if (esp_now_init() != ESP_OK)
  {

    Serial.println("ESP-NOW failed");

    while (true)
    {
      drive(0, 0);
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onReceive);

  // Every robot needs the broadcast peer: Car 3 sends commands and Cars 1/2
  // return observations on the same ESP-NOW channel.
  {

    esp_now_peer_info_t peer = {};

    memcpy(
        peer.peer_addr,
        broadcastMac,
        6);

    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK)
    {

      Serial.println("Peer failed");

      while (true)
      {
        drive(0, 0);
        delay(1000);
      }
    }

    // Only Car 3 exposes mission controls; it is the team's coordinator.
    if (IS_COORDINATOR)
    {
      startWebServer();
    }
  }

  Serial.printf(
      "Car %d (%s) ready\n",
      ROBOT_ID,
      roleToString(ROBOT_ROLE).c_str());
}

// Main loop
void loop()
{
  if (ULTRASONIC_SERIAL_TEST)
  {
    long distanceCm = readObstacleDistanceCm();
    if (distanceCm >= 999)
    {
      Serial.println("No echo - check TRIG/ECHO wiring and sensor power.");
    }
    else
    {
      Serial.printf("Distance: %ld cm\n", distanceCm);
    }

    delay(300);
    return;
  }

  uint32_t now = millis();
  if (BATTERY_MONITOR_ENABLED)
  {
    batteryVoltage = readBatteryVoltage();
  }
  lastObstacleDistanceCm = readObstacleDistanceCm();

  int16_t requestedLeft = 0;
  int16_t requestedRight = 0;
  RobotMode activeMode = MODE_IDLE;
  uint8_t activeModeCommandId = modeCommandId;

  if (IS_COORDINATOR)
  {
    int16_t webLeftNow;
    int16_t webRightNow;
    uint32_t webCommandAtNow;
    int newMode;
    portENTER_CRITICAL(&webMux);
    webLeftNow = webLeft;
    webRightNow = webRight;
    webCommandAtNow = webCommandAt;
    newMode = pendingMode;
    pendingMode = -1;
    portEXIT_CRITICAL(&webMux);
    // Read after the timestamp, so it can never be older and wrap around.
    now = millis();
    uint32_t sinceWebCommand = now - webCommandAtNow;

    if (newMode >= 0)
    {
      currentMode = static_cast<RobotMode>(newMode);
      ++modeCommandId;
      resetCoordination(now);
    }

    if (currentMode != MODE_IDLE && sinceWebCommand > OPERATOR_TIMEOUT_MS)
    {
      currentMode = MODE_IDLE;
      Serial.println("Operator page silent; team switched to Idle.");
    }

    bool explorerBlocked;
    uint32_t explorerReportedAt;
    portENTER_CRITICAL(&receiveMux);
    explorerBlocked = teamObservations[ROLE_EXPLORER].blocked;
    explorerReportedAt = teamObservations[ROLE_EXPLORER].receivedAt;
    portEXIT_CRITICAL(&receiveMux);
    // Only act on a fresh report; an offline explorer's last flag is stale.
    explorerBlocked = explorerBlocked &&
                      millis() - explorerReportedAt <= TEAM_TELEMETRY_TIMEOUT_MS;

    coordinateExplorer(now, explorerBlocked, explorerReportedAt);

    activeMode = currentMode;
    if (sinceWebCommand <= CONTROL_TIMEOUT_MS)
    {
      requestedLeft = webLeftNow;
      requestedRight = webRightNow;
    }

    if (now - lastRadioSend >= RADIO_INTERVAL_MS)
    {
      DrivePacket packet = {PACKET_MAGIC, PACKET_COMMAND, ROBOT_ID,
                            requestedLeft, requestedRight,
                            static_cast<uint8_t>(activeMode), modeCommandId};
      esp_now_send(broadcastMac, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      lastRadioSend = now;
    }
  }
  else
  {
    uint32_t receivedTime;
    portENTER_CRITICAL(&receiveMux);
    requestedLeft = receivedLeft;
    requestedRight = receivedRight;
    activeMode = receivedMode;
    activeModeCommandId = receivedModeCommandId;
    receivedTime = receivedAt;
    portEXIT_CRITICAL(&receiveMux);

    // A missing coordinator is always a stop, not a continuation of a mission.
    // Use a fresh millis(): receivedAt may be newer than "now", which was read
    // before pulseIn, and the subtraction would wrap around.
    if (millis() - receivedTime > CONTROL_TIMEOUT_MS)
    {
      activeMode = MODE_IDLE;
      requestedLeft = 0;
      requestedRight = 0;
    }
  }

  if (BATTERY_MONITOR_ENABLED && batteryVoltage < BATTERY_LOW_VOLTAGE)
  {
    avoidancePhase = AVOIDANCE_READY;
    targetLeft = 0;
    targetRight = 0;
  }
  else
  {
    applyRoleMode(activeMode, requestedLeft, requestedRight);
    if (activeMode != MODE_IDLE && activeMode != MODE_MAP && activeMode != MODE_COORDINATE &&
        activeMode != MODE_TURNAROUND)
    {
      int requestedRoleLeft = targetLeft;
      int requestedRoleRight = targetRight;
      applyObstacleAvoidance(now, requestedRoleLeft, requestedRoleRight);
    }
    else
    {
      avoidancePhase = AVOIDANCE_READY;
    }

    if (ROBOT_ROLE == ROLE_MAPPER)
    {
      updateObserverScan(now, activeMode, activeModeCommandId);
    }

    if (ROBOT_ROLE == ROLE_EXPLORER)
    {
      updateTurnaround(now, activeMode);
    }
  }

  // Timed turns must not be stretched by the speed ramp.
  if (avoidanceNeedsImmediateDrive() || observerScanTurning() ||
      turnaroundPhase == TURNAROUND_TURNING)
  {
    driveImmediately(targetLeft, targetRight);
  }
  else
  {
    drive(targetLeft, targetRight);
  }

  if (now - lastTelemetrySend >= TELEMETRY_INTERVAL_MS)
  {
    TelemetryPacket packet = {PACKET_MAGIC, PACKET_TELEMETRY, ROBOT_ID,
                              static_cast<uint8_t>(ROBOT_ROLE),
                              static_cast<uint16_t>(constrain(lastObstacleDistanceCm, 0, 999)),
                              static_cast<uint8_t>(avoidancePhase == AVOIDANCE_LOCKED), now};
    esp_now_send(broadcastMac, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    // Repeat the scan on every telemetry tick so a lost packet is replaced.
    if (ROBOT_ROLE == ROLE_MAPPER && observerReadings > 0)
    {
      ScanPacket scan = {PACKET_MAGIC, PACKET_SCAN, ROBOT_ID, observerScanId, observerReadings, {}};
      memcpy(scan.distanceCm, observerDistanceCm, sizeof(scan.distanceCm));
      esp_now_send(broadcastMac, reinterpret_cast<uint8_t *>(&scan), sizeof(scan));
    }

    lastTelemetrySend = now;
  }

  // Cars 1 and 2: the blue LED shows whether they are still on SWARM-X.
  if (!IS_COORDINATOR)
  {
    setStatusLed(WiFi.status() == WL_CONNECTED);
  }

  delay(5);
}
