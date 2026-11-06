// Runs updateObserverScan() from src/main.cpp (C types stripped) and tracks the
// car's heading from the motor commands it produces.
// Usage: node tools/sim/observer.js [src/main.cpp]
const src = require("fs").readFileSync(process.argv[2] || "src/main.cpp", "utf8").split("\r").join("");

const start = src.indexOf("void updateObserverScan(");
const fn = src.slice(start, src.indexOf("\n}\n", start) + 2)
  .replace("void updateObserverScan(uint32_t now, RobotMode mode, uint8_t commandId)",
           "function updateObserverScan(now, mode, commandId)")
  .replace(/static uint8_t scannedForCommandId = 0;/, "")
  .replace(/Serial\.(printf|println)\([\s\S]*?\);/g, "")
  .replace(/\b(int|bool)\s+/g, "let ");

const num = name => Number(src.match(new RegExp("const \\w+ " + name + "\\s*=\\s*(\\d+)"))[1]);
["OBSERVER_SCAN_STEPS", "ULTRASONIC_NO_ECHO_CM", "AVOIDANCE_TURN_SPEED", "AVOIDANCE_SETTLE_MS",
  "AVOIDANCE_TURN_90_MS"].forEach(c => globalThis[c] = num(c));
const phases = ["OBSERVER_IDLE", "OBSERVER_SETTLE", "OBSERVER_TURN", "OBSERVER_DONE"];
phases.forEach((p, i) => globalThis[p] = i);
globalThis.MODE_MAP = 5; globalThis.MODE_IDLE = 2;
const constrain = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

let observerPhase = 0, observerPhaseStartedAt = 0, observerScanId = 0, observerReadings = 0;
let observerDistanceCm = [0, 0, 0, 0], scannedForCommandId = 0;
let lastObstacleDistanceCm, targetLeft = 0, targetRight = 0, ultrasonicHasEchoed = true;
eval(fn);

// world: distance seen facing 0 / 90 / 180 / 270 degrees (turning right)
let heading = 0, t = 0;
function step(world, mode, id, ms) {
  let moved = 0;
  for (const end = t + ms; t < end; t += 30) {
    lastObstacleDistanceCm = world[Math.round(((heading % 360) + 360) % 360 / 90) % 4];
    targetLeft = 0; targetRight = 0;          // applyRoleMode(MODE_MAP) holds still
    updateObserverScan(t, mode, id);
    if (targetLeft > 0 && targetRight < 0) { heading += 90 * 30 / AVOIDANCE_TURN_90_MS; moved += 30; }
    if (targetLeft !== 0 && !(targetLeft > 0 && targetRight < 0)) throw new Error("unexpected motor command");
  }
  return moved;
}

let failed = 0;
const check = (label, ok, detail) => { if (!ok) failed++; console.log(`${ok ? "PASS" : "FAIL"}  ${label}   [${detail}]`); };
const world = [120, 40, 999, 75];

step(world, MODE_MAP, 1, 3000);
check("4 readings in order front/right/back/left", observerDistanceCm.join() === world.join(), observerDistanceCm.join());
check("scan finished and holding", observerPhase === OBSERVER_DONE && observerReadings === 4,
  phases[observerPhase] + ", readings " + observerReadings);
check("ends at its starting heading", Math.abs(heading - 360) < 1e-6, "turned " + heading.toFixed(1) + " deg");
check("no movement while holding", step(world, MODE_MAP, 1, 3000) === 0, "phase " + phases[observerPhase]);
check("same Map command does not rescan", observerScanId === 1, "scanId " + observerScanId);

step(world, MODE_MAP, 2, 3000);
check("pressing Map again rescans", observerScanId === 2 && observerReadings === 4 && Math.abs(heading - 720) < 1e-6,
  "scanId " + observerScanId + ", turned " + heading.toFixed(0));

step(world, MODE_MAP, 3, 500);
const midReadings = observerReadings;
step(world, MODE_IDLE, 3, 100);
check("leaving Map stops the scan", observerPhase === OBSERVER_IDLE, "had " + midReadings + " readings, phase " + phases[observerPhase]);

ultrasonicHasEchoed = false; heading = 0;
step([999, 999, 999, 999], MODE_MAP, 4, 3000);
check("unplugged sensor reports 0 (blocked), not open", observerDistanceCm.every(d => d === 0), observerDistanceCm.join());

console.log(failed ? `${failed} FAILED` : "ALL PASS");
process.exitCode = failed ? 1 : 0;
