
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// Change only this ID for each car
// 1 = Leader, 2 = Follower, 3 = Follower
#define ROBOT_ID 1

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
const uint32_t CONTROL_TIMEOUT_MS = 800;
const uint32_t RADIO_INTERVAL_MS = 100;

const char *AP_NAME = "SWARM-X-Control";
const char *AP_PASSWORD = "swarmx12345";

// ESP-NOW
const uint32_t PACKET_MAGIC = 0x53574152;

uint8_t broadcastMac[6] = {
    0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF};

WebServer server(80);

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
const int OBSTACLE_DISTANCE_CM = 45;
const int OBSTACLE_CLEAR_DISTANCE_CM = 55;
const int MAX_AVOIDANCE_ATTEMPTS = 2;
const int AVOIDANCE_REVERSE_SPEED = 120;
const int AVOIDANCE_TURN_SPEED = 120;
const uint32_t AVOIDANCE_STOP_MS = 200;
const uint32_t AVOIDANCE_REVERSE_MS = 600;
const uint32_t AVOIDANCE_TURN_STOP_MS = 150;
const uint32_t AVOIDANCE_TURN_MS = 600;
const uint32_t MOTOR_RAMP_INTERVAL_MS = 20;
const int MOTOR_ACCEL_STEP = 5;
const int MOTOR_BRAKE_STEP = 20;

enum RobotMode
{
  MODE_MANUAL = 0,
  MODE_AVOID = 1,
  MODE_IDLE = 2,
  MODE_LINEFOLLOW = 3
};

enum AvoidancePhase
{
  AVOIDANCE_READY,
  AVOIDANCE_STOPPING,
  AVOIDANCE_REVERSING,
  AVOIDANCE_TURN_STOPPING,
  AVOIDANCE_TURNING,
  AVOIDANCE_LOCKED
};

volatile RobotMode currentMode = MODE_MANUAL;
volatile int16_t commandedLeft = 0;
volatile int16_t commandedRight = 0;
float batteryVoltage = 0.0f;
int lastObstacleDistanceCm = 999;
int lastLineTurn = 0;
AvoidancePhase avoidancePhase = AVOIDANCE_READY;
uint32_t avoidancePhaseStartedAt = 0;
int avoidanceAttempts = 0;
int avoidanceTurnDirection = 1;

// Wireless packet
struct __attribute__((packed)) DrivePacket
{
  uint32_t magic;
  uint8_t source;
  int16_t left;
  int16_t right;
};

// Motor variables
int16_t targetLeft = 0;
int16_t targetRight = 0;

int16_t appliedLeft = 0;
int16_t appliedRight = 0;

volatile int16_t receivedLeft = 0;
volatile int16_t receivedRight = 0;
volatile uint32_t receivedAt = 0;

uint32_t lastBrowserCommand = 0;
uint32_t lastRadioSend = 0;

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

  if (ROBOT_ID == 1 ||
      length != sizeof(DrivePacket))
  {
    return;
  }

  DrivePacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != PACKET_MAGIC ||
      packet.source != 1)
  {
    return;
  }

  portENTER_CRITICAL(&receiveMux);

  receivedLeft = constrain(packet.left, -255, 255);
  receivedRight = constrain(packet.right, -255, 255);
  receivedAt = millis();

  portEXIT_CRITICAL(&receiveMux);
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
    return 999;
  }

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
    default:
      return "manual";
  }
}

String avoidancePhaseToString()
{
  switch (avoidancePhase)
  {
    case AVOIDANCE_STOPPING:
      return "Stopping for obstacle";
    case AVOIDANCE_REVERSING:
      return "Reversing from obstacle";
    case AVOIDANCE_TURN_STOPPING:
      return "Pausing before turn";
    case AVOIDANCE_TURNING:
      return "Turning to avoid obstacle";
    case AVOIDANCE_LOCKED:
      return "Blocked - release controls or clear path";
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
  bool obstacleClose = lastObstacleDistanceCm > 0 &&
                       lastObstacleDistanceCm < OBSTACLE_DISTANCE_CM;
  bool pathClear = lastObstacleDistanceCm >= OBSTACLE_CLEAR_DISTANCE_CM;
  bool recoveryStillBlocked = avoidanceAttempts > 0 &&
                              !pathClear &&
                              lastObstacleDistanceCm < 999;

  if (!forwardRequested)
  {
    avoidancePhase = AVOIDANCE_READY;
    avoidanceAttempts = 0;
    targetLeft = requestedLeft;
    targetRight = requestedRight;
    return;
  }

  if (avoidancePhase == AVOIDANCE_LOCKED)
  {
    if (pathClear)
    {
      avoidancePhase = AVOIDANCE_READY;
      avoidanceAttempts = 0;
    }
    else
    {
      targetLeft = 0;
      targetRight = 0;
      return;
    }
  }

  if (avoidancePhase == AVOIDANCE_READY)
  {
    if (obstacleClose || recoveryStillBlocked)
    {
      if (avoidanceAttempts >= MAX_AVOIDANCE_ATTEMPTS)
      {
        avoidancePhase = AVOIDANCE_LOCKED;
        targetLeft = 0;
        targetRight = 0;
        return;
      }

      ++avoidanceAttempts;
      avoidancePhase = AVOIDANCE_STOPPING;
      avoidancePhaseStartedAt = now;
      avoidanceTurnDirection = (avoidanceAttempts % 2 == 1) ? 1 : -1;
      Serial.printf("Obstacle at %d cm; recovery attempt %d/%d\n",
                    lastObstacleDistanceCm,
                    avoidanceAttempts,
                    MAX_AVOIDANCE_ATTEMPTS);
    }
    else if (pathClear)
    {
      avoidanceAttempts = 0;
    }
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
        avoidancePhase = AVOIDANCE_TURN_STOPPING;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_TURN_STOPPING:
      targetLeft = 0;
      targetRight = 0;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_TURN_STOP_MS)
      {
        avoidancePhase = AVOIDANCE_TURNING;
        avoidancePhaseStartedAt = now;
      }
      break;

    case AVOIDANCE_TURNING:
      targetLeft = -avoidanceTurnDirection * AVOIDANCE_TURN_SPEED;
      targetRight = avoidanceTurnDirection * AVOIDANCE_TURN_SPEED;
      if (now - avoidancePhaseStartedAt >= AVOIDANCE_TURN_MS)
      {
        targetLeft = 0;
        targetRight = 0;
        avoidancePhase = AVOIDANCE_READY;
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

bool avoidanceNeedsImmediateDrive()
{
  return avoidancePhase == AVOIDANCE_STOPPING ||
         avoidancePhase == AVOIDANCE_REVERSING ||
         avoidancePhase == AVOIDANCE_TURN_STOPPING ||
         avoidancePhase == AVOIDANCE_TURNING ||
         avoidancePhase == AVOIDANCE_LOCKED;
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
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 10px;
    margin-top: 12px;
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
      <p class="subtitle">Leader + Followers | ESP-NOW</p>
    </div>

    <div class="panel">
      <div id="status" class="status">Ready - waiting for command</div>

      <div class="stats">
        <div class="stat">
          <span class="stat-label">Battery</span>
          <span id="batteryValue" class="stat-value">N/A</span>
        </div>
        <div class="stat">
          <span class="stat-label">Obstacle</span>
          <span id="obstacleValue" class="stat-value">-- cm</span>
        </div>
        <div class="stat">
          <span class="stat-label">Mode</span>
          <span id="modeValue" class="stat-value">Manual</span>
        </div>
      </div>
    </div>

    <div class="mode-bar">
      <button class="mode-btn active" data-mode="manual">Manual</button>
      <button class="mode-btn" data-mode="avoid">Avoid</button>
      <button class="mode-btn" data-mode="line">Line</button>
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
        Forward commands in any drive mode stop, reverse, and turn for obstacles under 45 cm.<br>
        It tries twice, then stays stopped if still blocked.<br>
        One front sensor cannot check whether the side path is clear.<br>
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
    const joystickEl = document.getElementById('joystick');
    const stickEl = document.getElementById('stick');
    const modeButtons = [...document.querySelectorAll('.mode-btn')];

    let activeMode = 'manual';
    let busy = false;
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

    function setMode(mode) {
      activeMode = mode;
      modeButtons.forEach(btn => btn.classList.toggle('active', btn.dataset.mode === mode));
      modeValue.textContent = mode === 'line' ? 'Line' : mode.charAt(0).toUpperCase() + mode.slice(1);
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
      if (busy) return;
      busy = true;
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), 550);

      fetch('/drive?l=' + Math.round(left) + '&r=' + Math.round(right) + '&m=' + activeMode, {
        cache: 'no-store',
        signal: controller.signal
      })
      .then(resp => resp.text())
      .then(text => {
        if (text && text.trim() === 'OK') {
          status.textContent = 'Connected - motors active';
          status.className = 'status';
        } else {
          status.textContent = 'Command rejected';
          status.className = 'status danger';
        }
      })
      .catch(() => {
        status.textContent = 'Connection problem - motors should stop';
        status.className = 'status danger';
      })
      .finally(() => {
        clearTimeout(timeout);
        busy = false;
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
      stickState.y = -ny;
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

    setInterval(() => {
      fetch('/status', { cache: 'no-store' })
        .then(r => r.json())
        .then(data => {
          if (!data.batteryEnabled) {
            batteryValue.textContent = 'N/A';
          } else if (typeof data.battery === 'number') {
            batteryValue.textContent = Number(data.battery).toFixed(1) + 'V';
          }
          if (typeof data.distance === 'number') obstacleValue.textContent = data.distance + ' cm';
          if (data.mode) modeValue.textContent = data.mode === 'line' ? 'Line' : data.mode.charAt(0).toUpperCase() + data.mode.slice(1);
          if (data.status) {
            status.textContent = data.status;
            status.className = 'status ' + (data.warn ? 'warning' : '');
          }
        })
        .catch(() => {});

      if (!document.hidden) {
        sendCurrentAxes();
      }
    }, 500);

    speedValue.textContent = speed.value;
    setMode('manual');
    updateStick();
  </script>
</body>
</html>
)SWARMHTML";

// ESP32 setup
void setup()
{

  Serial.begin(115200);

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

  if (ROBOT_ID == 1)
  {

    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);

    if (!WiFi.softAP(
            AP_NAME,
            AP_PASSWORD,
            RADIO_CHANNEL))
    {

      Serial.println("WiFi failed");

      while (true)
      {
        drive(0, 0);
        delay(1000);
      }
    }

    Serial.println("WiFi ready");
    Serial.println(AP_NAME);
    Serial.println(WiFi.softAPIP());
  }
  else
  {

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);

    esp_wifi_set_channel(
        RADIO_CHANNEL,
        WIFI_SECOND_CHAN_NONE);
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

  // Leader configuration
  if (ROBOT_ID == 1)
  {

    esp_now_peer_info_t peer = {};

    memcpy(
        peer.peer_addr,
        broadcastMac,
        6);

    peer.channel = RADIO_CHANNEL;
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

    // Web dashboard
    server.on("/", HTTP_GET, []()
              { server.send_P(
                    200,
                    "text/html",
                    PAGE); });

    // Receive motor commands from laptop
    server.on("/drive", HTTP_GET, []()
              {
                if (!server.hasArg("l") ||
                    !server.hasArg("r"))
                {

                  server.send(
                      400,
                      "text/plain",
                      "Missing command");

                  return;
                }

                commandedLeft = constrain(
                    server.arg("l").toInt(),
                    -255,
                    255);

                commandedRight = constrain(
                    server.arg("r").toInt(),
                    -255,
                    255);

                if (server.hasArg("m"))
                {
                  currentMode = parseMode(server.arg("m"));
                }

                lastBrowserCommand = millis();

                Serial.printf(
                    "HTTP drive L:%d R:%d mode=%s\n",
                    commandedLeft,
                    commandedRight,
                    modeToString(currentMode).c_str());

                server.send(
                    200,
                    "text/plain",
                    "OK"); });

    server.on("/mode", HTTP_GET, []()
              {
                if (!server.hasArg("m"))
                {
                  server.send(400, "text/plain", "Missing mode");
                  return;
                }

                currentMode = parseMode(server.arg("m"));
                lastBrowserCommand = millis();
                server.send(200, "text/plain", modeToString(currentMode)); });

    server.on("/status", HTTP_GET, []()
              {
                String payload = "{\"batteryEnabled\":" +
                                 String(BATTERY_MONITOR_ENABLED ? "true" : "false") +
                                 ",\"battery\":" +
                                 (BATTERY_MONITOR_ENABLED ? String(batteryVoltage, 1) : String("null")) +
                                 ",\"distance\":" + String(lastObstacleDistanceCm) +
                                 ",\"mode\":\"" + modeToString(currentMode) +
                                 "\",\"avoidance\":\"" + avoidancePhaseToString() + "\"";

                if (BATTERY_MONITOR_ENABLED && batteryVoltage < BATTERY_LOW_VOLTAGE)
                {
                  payload += ",\"warn\":true,\"status\":\"Low battery - motors disabled\"";
                }
                else if (avoidancePhase == AVOIDANCE_LOCKED)
                {
                  payload += ",\"warn\":true,\"status\":\"Blocked - release controls or clear path\"";
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
                server.send(200, "application/json", payload);
              });

    server.onNotFound([]()
                      { server.send(
                            404,
                            "text/plain",
                            "Not found"); });

    server.begin();
  }

  Serial.printf(
      "Robot %d ready\n",
      ROBOT_ID);
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

  if (ROBOT_ID == 1)
  {

    // Leader
    server.handleClient();

    uint32_t now = millis();
    if (BATTERY_MONITOR_ENABLED)
    {
      batteryVoltage = readBatteryVoltage();
    }
    lastObstacleDistanceCm = readObstacleDistanceCm();

    // Stop if laptop disconnects
    if (now - lastBrowserCommand >
        CONTROL_TIMEOUT_MS)
    {

      commandedLeft = 0;
      commandedRight = 0;
    }

    if (BATTERY_MONITOR_ENABLED &&
        batteryVoltage < BATTERY_LOW_VOLTAGE)
    {
      avoidancePhase = AVOIDANCE_READY;
      avoidanceAttempts = 0;
      targetLeft = 0;
      targetRight = 0;
    }
    else if (currentMode == MODE_IDLE)
    {
      avoidancePhase = AVOIDANCE_READY;
      avoidanceAttempts = 0;
      targetLeft = 0;
      targetRight = 0;
    }
    else if (currentMode == MODE_LINEFOLLOW)
    {
      followLine();
    }
    else
    {
      targetLeft = commandedLeft;
      targetRight = commandedRight;
    }

    if ((!BATTERY_MONITOR_ENABLED || batteryVoltage >= BATTERY_LOW_VOLTAGE) &&
        currentMode != MODE_IDLE)
    {
      int requestedLeft = targetLeft;
      int requestedRight = targetRight;
      applyObstacleAvoidance(now, requestedLeft, requestedRight);
    }

    if (avoidanceNeedsImmediateDrive())
    {
      driveImmediately(targetLeft, targetRight);
    }
    else
    {
      drive(targetLeft, targetRight);
    }

    // Broadcast commands to followers
    if (now - lastRadioSend >=
        RADIO_INTERVAL_MS)
    {

      DrivePacket packet = {
          PACKET_MAGIC,
          1,
          targetLeft,
          targetRight};

      esp_now_send(
          broadcastMac,
          (uint8_t *)&packet,
          sizeof(packet));

      lastRadioSend = now;
    }
  }
  else
  {

    // Follower
    int16_t left, right;
    uint32_t receivedTime;

    portENTER_CRITICAL(&receiveMux);

    left = receivedLeft;
    right = receivedRight;
    receivedTime = receivedAt;

    portEXIT_CRITICAL(&receiveMux);

    // Stop if leader disconnects
    if (millis() - receivedTime >
        CONTROL_TIMEOUT_MS)
    {

      drive(0, 0);
    }
    else
    {

      drive(left, right);
    }
  }

  delay(5);
}
