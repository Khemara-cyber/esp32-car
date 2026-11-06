// Runs Car 3's coordinateExplorer()/resetCoordination() and Car 1's
// updateTurnaround() from src/main.cpp (C types stripped), connected by a
// model of the radio (commands every 100 ms, telemetry every 500 ms).
// Usage: node tools/sim/coordinator.js [src/main.cpp]
const src = require("fs").readFileSync(process.argv[2] || "src/main.cpp", "utf8").split("\r").join("");
const grabFn = sig => {
  const i = src.indexOf(sig);
  if (i < 0) throw new Error("not found: " + sig);
  return src.slice(i, src.indexOf("\n}\n", i) + 2);
};
const js = [
  grabFn("void updateTurnaround(uint32_t now, RobotMode mode)"),
  grabFn("void resetCoordination(uint32_t now)"),
  grabFn("void coordinateExplorer(uint32_t now, bool explorerBlocked, uint32_t explorerReportedAt)"),
].join("\n")
  .replace(/void (\w+)\(([^)]*)\)/g, (_, name, args) =>
    `function ${name}(${args.split(",").map(a => a.trim().split(/\s+/).pop()).join(", ")})`)
  .replace(/static_cast<int32_t>\(([^)]*)\)/g, "(($1)|0)")
  .replace(/Serial\.(printf|println)\([\s\S]*?\);/g, "")
  .replace(/\b(bool|int)\s+/g, "let ");

const num = name => {
  const m = src.match(new RegExp("const \\w+ " + name + "\\s*=\\s*([^;]+);"));
  return Function("AVOIDANCE_TURN_90_MS", "return " + m[1])(globalThis.AVOIDANCE_TURN_90_MS);
};
globalThis.AVOIDANCE_TURN_90_MS = num("AVOIDANCE_TURN_90_MS");
["AVOIDANCE_TURN_SPEED", "MAX_EXPLORER_RECOVERIES", "EXPLORER_PROGRESS_MS", "TURNAROUND_RESUME_MS"]
  .forEach(c => globalThis[c] = num(c));
Object.assign(globalThis, { MODE_IDLE: 2, MODE_EXPLORE: 4, MODE_COORDINATE: 6, MODE_TURNAROUND: 7,
  TURNAROUND_NONE: 0, TURNAROUND_TURNING: 1, TURNAROUND_DONE: 2 });
const NAMES = { 2: "IDLE", 4: "EXPLORE", 6: "COORDINATE", 7: "TURNAROUND" };

// Shared C globals
let currentMode, explorerRecoveries, explorerNeedsOperator, coordinatorPhaseStartedAt;
let turnaroundPhase, turnaroundStartedAt, targetLeft, targetRight;
eval(js);

const SCAN_MS = 2200;   // Car 1's stop/reverse/scan sequence before it can report blocked

// deadEnd(heading) says whether Car 1 hits a dead end when exploring that way.
function run(deadEnd, ms, { staleReportAt = null } = {}) {
  currentMode = MODE_EXPLORE; turnaroundPhase = TURNAROUND_NONE;
  resetCoordination(0);
  let heading = 0, explorerMode = MODE_IDLE, blockedSince = null, locked = false;
  let report = { blocked: false, at: 0 }, modes = [NAMES[currentMode]], turnbacks = 0;
  if (staleReportAt !== null) report = { blocked: true, at: staleReportAt };

  for (let t = 30; t <= ms; t += 30) {
    // Car 3
    const before = currentMode;
    coordinateExplorer(t, report.blocked, report.at);
    if (currentMode !== before) {
      modes.push(NAMES[currentMode] + "@" + (t / 1000).toFixed(1) + "s");
      if (currentMode === MODE_TURNAROUND) turnbacks++;
    }
    if (t % 100 === 0) explorerMode = currentMode;           // command broadcast

    // Car 1
    targetLeft = targetRight = 0;
    if (explorerMode === MODE_EXPLORE) {
      if (deadEnd(heading)) {
        blockedSince ??= t;
        locked = t - blockedSince >= SCAN_MS;                // avoidance ends LOCKED
      }
    } else {
      blockedSince = null; locked = false;                   // loop() resets avoidance
    }
    updateTurnaround(t, explorerMode);
    if (targetLeft > 0 && targetRight < 0) heading += 90 * 30 / AVOIDANCE_TURN_90_MS;
    if (t % 500 === 0) report = { blocked: locked, at: t };  // telemetry
  }
  return { modes, turnbacks, heading: Math.round(heading) % 360 };
}

let failed = 0;
const check = (label, ok, detail) => { if (!ok) failed++; console.log(`${ok ? "PASS" : "FAIL"}  ${label}\n      ${detail}`); };

let r = run(h => Math.round(h) % 360 === 0, 15000);
check("Dead end ahead, open behind: one turn-back, then keeps exploring",
  r.turnbacks === 1 && currentMode === MODE_EXPLORE && r.heading === 180 && !explorerNeedsOperator,
  r.modes.join(" -> ") + ` | heading ${r.heading}, final ${NAMES[currentMode]}`);
check("...and the dead end is forgotten after exploring freely", explorerRecoveries === 0,
  "explorerRecoveries " + explorerRecoveries);

r = run(() => true, 20000);
check("Dead ends every way: 2 turn-backs, then team paused for the operator",
  r.turnbacks === 2 && currentMode === MODE_COORDINATE && explorerNeedsOperator,
  r.modes.join(" -> ") + ` | needsOperator ${explorerNeedsOperator}`);

resetCoordination(20030);
check("A mode button press clears the operator request", !explorerNeedsOperator && explorerRecoveries === 0,
  `needsOperator ${explorerNeedsOperator}, recoveries ${explorerRecoveries}`);

r = run(() => false, 3000, { staleReportAt: -10 });
check("A blocked report from before Explore started is ignored", r.turnbacks === 0 && currentMode === MODE_EXPLORE,
  r.modes.join(" -> "));

console.log(failed ? `${failed} FAILED` : "ALL PASS");
process.exitCode = failed ? 1 : 0;
