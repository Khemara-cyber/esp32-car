
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

  if (left == appliedLeft &&
      right == appliedRight)
  {
    return;
  }

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
<meta name="viewport"
content="width=device-width, initial-scale=1">

<title>SWARM-X Remote</title>

<style>
* {
  box-sizing: border-box;
}

body {
  margin: 0;
  background: #101621;
  color: white;
  font-family: Arial, sans-serif;
  display: flex;
  justify-content: center;
  align-items: center;
  min-height: 100vh;
}

.container {
  width: 100%;
  max-width: 500px;
  padding: 25px;
  text-align: center;
}

h1 {
  margin-bottom: 5px;
}

.subtitle {
  color: #9daec4;
  margin-bottom: 20px;
}

.status {
  background: #1b2637;
  border: 1px solid #344358;
  border-radius: 12px;
  padding: 15px;
  margin-bottom: 20px;
}

.controls {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 10px;
}

button {
  height: 80px;
  border: 1px solid #46556c;
  border-radius: 14px;
  background: #27374e;
  color: white;
  font-size: 28px;
  cursor: pointer;
  touch-action: none;
  user-select: none;
}

button:active,
button.active {
  background: #267e9c;
}

.stop {
  background: #943a45;
  font-size: 18px;
  font-weight: bold;
}

.speed-control {
  margin-top: 25px;
}

input[type=range] {
  width: 100%;
  margin-top: 15px;
}

.info {
  color: #91a6c0;
  font-size: 13px;
  margin-top: 20px;
  line-height: 1.7;
}
</style>
</head>

<body>

<div class="container">

<h1>SWARM-X Remote</h1>

<p class="subtitle">
Leader + 2 Followers | ESP-NOW
</p>

<div class="status" id="status">
Ready - Hold a direction to drive
</div>

<div class="controls">

<button data-f="1" data-t="-1">↖</button>
<button data-f="1" data-t="0">↑</button>
<button data-f="1" data-t="1">↗</button>

<button data-f="0" data-t="-1">←</button>
<button class="stop" id="stop">STOP</button>
<button data-f="0" data-t="1">→</button>

<button data-f="-1" data-t="-1">↙</button>
<button data-f="-1" data-t="0">↓</button>
<button data-f="-1" data-t="1">↘</button>

</div>

<div class="speed-control">
Motor Speed:
<b id="speedValue">200</b> / 255

<input
  type="range"
  id="speed"
  min="100"
  max="255"
  step="5"
  value="200"
>
</div>

<p class="info">
Hold buttons or use W A S D / Arrow Keys.<br>
Forward + Left/Right supports steering.<br>
Release controls to stop all cars.
</p>

</div>

<script>

const buttons = [
  ...document.querySelectorAll("[data-f]")
];

const speed = document.getElementById("speed");
const speedValue = document.getElementById("speedValue");
const status = document.getElementById("status");

let pointer = null;
let keys = new Set();
let busy = false;

const directions = {
  ArrowUp: "up",
  KeyW: "up",
  ArrowDown: "down",
  KeyS: "down",
  ArrowLeft: "left",
  KeyA: "left",
  ArrowRight: "right",
  KeyD: "right"
};

function clearButtons() {
  buttons.forEach(button => {
    button.classList.remove("active");
  });
}

function axes() {

  if (pointer) {
    return pointer;
  }

  return {
    f: Number(keys.has("up")) -
       Number(keys.has("down")),

    t: Number(keys.has("right")) -
       Number(keys.has("left"))
  };
}


function wheelSpeeds() {
  const {f, t} = axes();
  const s = Number(speed.value);

  let left = 0;
  let right = 0;

  if (f !== 0) {
    left = f * s;
    right = f * s;

    // Forward/backward steering
    if (t > 0) {
      left = f * Math.round(s * 0.45);
    }

    if (t < 0) {
      right = f * Math.round(s * 0.45);
    }

  } else if (t !== 0) {
    // Correct reversed A/D rotation
    left = -t * s;
    right = t * s;
  }

  return [left, right];
}


async function sendOnce() {

  // Prevent overlapping requests
  if (busy) return;

  const [left, right] = wheelSpeeds();

  busy = true;

  const controller = new AbortController();

  const timeout = setTimeout(() => {
    controller.abort();
  }, 550);

  try {

    const response = await fetch(
      "/drive?l=" + left + "&r=" + right,
      {
        cache: "no-store",
        signal: controller.signal
      }
    );

    if (!response.ok) {
      throw new Error("HTTP error");
    }

    const text = await response.text();

    if (text.trim() !== "OK") {
      throw new Error("Invalid response");
    }

    status.textContent =
      "Connected - Sending commands";

    status.style.color = "#89d9bd";

  } catch (error) {

    status.textContent =
      "Connection problem - Motors should stop";

    status.style.color = "#ffb1b1";

  } finally {

    clearTimeout(timeout);
    busy = false;
  }
}

function stop() {

  pointer = null;
  keys.clear();

  clearButtons();
  sendOnce();
}

buttons.forEach(button => {

  button.addEventListener("pointerdown", e => {

    e.preventDefault();

    pointer = {
      f: Number(button.dataset.f),
      t: Number(button.dataset.t)
    };

    clearButtons();
    button.classList.add("active");

    try {
      button.setPointerCapture(e.pointerId);
    } catch (error) {}

    sendOnce();
  });

  [
    "pointerup",
    "pointercancel",
    "lostpointercapture"
  ].forEach(eventName => {

    button.addEventListener(eventName, () => {

      pointer = null;
      clearButtons();
      sendOnce();
    });

  });

});

document.getElementById("stop")
.addEventListener("pointerdown", e => {

  e.preventDefault();
  stop();

});

window.addEventListener("keydown", e => {

  if (directions[e.code]) {

    e.preventDefault();
    keys.add(directions[e.code]);
  }

  if (e.code === "Space") {

    e.preventDefault();
    stop();
  }

});

window.addEventListener("keyup", e => {

  if (directions[e.code]) {

    e.preventDefault();
    keys.delete(directions[e.code]);
    sendOnce();
  }

});

window.addEventListener("blur", stop);

document.addEventListener(
  "visibilitychange",
  () => {
    if (document.hidden) stop();
  }
);

speed.addEventListener("input", () => {

  speedValue.textContent = speed.value;
  sendOnce();

});

// Keep commands updated
setInterval(() => {

  if (!document.hidden) {
    sendOnce();
  }

}, 180);

</script>

</body>
</html>
)SWARMHTML";

// ESP32 setup
void setup()
{

  Serial.begin(115200);

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

                targetLeft = constrain(
                    server.arg("l").toInt(),
                    -255,
                    255);

                targetRight = constrain(
                    server.arg("r").toInt(),
                    -255,
                    255);

                lastBrowserCommand = millis();

                // Apply movement immediately
                drive(targetLeft, targetRight);

                Serial.printf(
                    "HTTP drive L:%d R:%d\n",
                    targetLeft,
                    targetRight);

                server.send(
                    200,
                    "text/plain",
                    "OK"); });

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

  if (ROBOT_ID == 1)
  {

    // Leader
    server.handleClient();

    uint32_t now = millis();

    // Stop if laptop disconnects
    if (now - lastBrowserCommand >
        CONTROL_TIMEOUT_MS)
    {

      targetLeft = 0;
      targetRight = 0;
    }

    drive(targetLeft, targetRight);

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
