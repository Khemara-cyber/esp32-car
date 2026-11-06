// Runs applyObstacleAvoidance() from src/main.cpp in Node, with only the C type
// keywords stripped, against a simple world (distance ahead / left / right).
// Usage: node tools/sim/avoid.js [src/main.cpp]
const fs = require("fs");
const src = fs.readFileSync(process.argv[2] || "src/main.cpp", "utf8").split("\r").join("");

const start = src.indexOf("void applyObstacleAvoidance(");
const fn = src.slice(start, src.indexOf("\n}\n", start) + 2)
  .replace("void applyObstacleAvoidance(uint32_t now, int requestedLeft, int requestedRight)",
           "function applyObstacleAvoidance(now, requestedLeft, requestedRight)")
  .replace(/\b(bool|int)\s+/g, "let ")
  .replace(/Serial\.(printf|println)\([\s\S]*?\);/g, "");

const num = name => Number(src.match(new RegExp("const \\w+ " + name + "\\s*=\\s*(\\d+)"))[1]);
const consts = ["OBSTACLE_DISTANCE_CM", "OBSTACLE_CLEAR_DISTANCE_CM", "ULTRASONIC_NO_ECHO_CM",
  "ULTRASONIC_MAX_RANGE_CM", "AVOIDANCE_REVERSE_SPEED", "AVOIDANCE_TURN_SPEED", "AVOIDANCE_STOP_MS",
  "AVOIDANCE_REVERSE_MS", "AVOIDANCE_SETTLE_MS", "AVOIDANCE_TURN_90_MS"];
consts.forEach(c => globalThis[c] = num(c));
const phases = ["AVOIDANCE_READY", "AVOIDANCE_STOPPING", "AVOIDANCE_REVERSING", "AVOIDANCE_SCAN_LEFT",
  "AVOIDANCE_READ_LEFT", "AVOIDANCE_SCAN_RIGHT", "AVOIDANCE_READ_RIGHT", "AVOIDANCE_CHOOSE_PATH",
  "AVOIDANCE_LOCKED"];
phases.forEach((p, i) => globalThis[p] = i);

let avoidancePhase, avoidancePhaseStartedAt, leftScanDistanceCm, rightScanDistanceCm;
let lastObstacleDistanceCm, targetLeft, targetRight, ultrasonicHasEchoed;
eval(fn);

// world: distance seen when facing front / left / right (999 = no echo)
function run(label, world, { everEchoed = true, expect }) {
  avoidancePhase = 0; avoidancePhaseStartedAt = 0; leftScanDistanceCm = rightScanDistanceCm = 0;
  ultrasonicHasEchoed = everEchoed;
  let heading = "front", last = -1, log = [];
  for (let t = 0; t <= 4000; t += 30) {          // ~30 ms loop: 25 ms pulseIn + 5 ms delay
    lastObstacleDistanceCm = world[heading];
    const before = avoidancePhase;
    applyObstacleAvoidance(t, 165, 165);         // Explore mode requests 165/165
    if (avoidancePhase === AVOIDANCE_READ_LEFT) heading = "left";
    if (avoidancePhase === AVOIDANCE_READ_RIGHT) heading = "right";
    if (before === AVOIDANCE_CHOOSE_PATH && avoidancePhase === AVOIDANCE_READY &&
        leftScanDistanceCm > rightScanDistanceCm) heading = "left";
    if (avoidancePhase !== last) { log.push(phases[avoidancePhase].slice(10)); last = avoidancePhase; }
  }
  const driving = targetLeft > 0 && targetRight > 0;
  const result = driving ? `driving, facing ${heading}` : "stopped";
  const ok = result === expect;
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}\n      ${log.join(" -> ")}\n      result: ${result} (expected: ${expect})`);
  return ok;
}

const results = [
  run("Open space, no echo ahead", { front: 999 }, { expect: "driving, facing front" }),
  run("Wall 200 cm ahead", { front: 200 }, { expect: "driving, facing front" }),
  run("Wall 30 cm, left 150 cm, right 40 cm", { front: 30, left: 150, right: 40 }, { expect: "driving, facing left" }),
  run("Wall 30 cm, left wide open (no echo), right 40 cm", { front: 30, left: 999, right: 40 }, { expect: "driving, facing left" }),
  run("Wall 30 cm, left 40 cm, right 120 cm", { front: 30, left: 40, right: 120 }, { expect: "driving, facing right" }),
  run("Dead end: 30 / 40 / 40 cm", { front: 30, left: 40, right: 40 }, { expect: "stopped" }),
  run("Sensor unplugged (never echoed)", { front: 999, left: 999, right: 999 }, { everEchoed: false, expect: "stopped" }),
];
const failed = results.filter(r => !r).length;
console.log(failed ? `\n${failed} FAILED` : "\nALL PASS");
process.exit(failed ? 1 : 0);
