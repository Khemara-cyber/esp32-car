// Runs Car 3's followSpeed() from src/main.cpp (C types stripped) through the
// same speed ramp as drive(), against a moving Car 1.
// Model assumption (calibrate on the real cars): speed in cm/s = PWM * 0.2,
// and PWM below 80 does not move the car.
// Usage: node tools/sim/follow.js [src/main.cpp]
const src = require("fs").readFileSync(process.argv[2] || "src/main.cpp", "utf8").split("\r").join("");
const i = src.indexOf("int followSpeed(int distanceCm)");
const fn = src.slice(i, src.indexOf("\n}\n", i) + 2).replace("int followSpeed(int distanceCm)", "function followSpeed(distanceCm)");

const num = name => {
  const m = src.match(new RegExp("const \\w+ " + name + "\\s*=\\s*([^;]+);"));
  return Function(...Object.keys(globalThis.C), "return " + m[1])(...Object.values(globalThis.C));
};
globalThis.C = {};
["EXPLORE_SPEED", "FOLLOW_TARGET_CM", "FOLLOW_TOO_CLOSE_CM", "FOLLOW_LOST_CM", "FOLLOW_MIN_SPEED",
  "FOLLOW_MAX_SPEED", "FOLLOW_BACKOFF_SPEED", "ULTRASONIC_NO_ECHO_CM", "MOTOR_ACCEL_STEP", "MOTOR_BRAKE_STEP",
  "AVOIDANCE_REVERSE_SPEED"].forEach(c => { C[c] = num(c); globalThis[c] = C[c]; });
const map = (x, a, b, c, d) => Math.trunc((x - a) * (d - c) / (b - a) + c);   // Arduino map()
eval(fn);

// drive()'s ramp: accelerate by MOTOR_ACCEL_STEP, brake by MOTOR_BRAKE_STEP, every 20 ms
function ramp(current, target) {
  if (current !== 0 && target !== 0 && (current < 0) !== (target < 0)) target = 0;
  const step = Math.abs(target) < Math.abs(current) ? MOTOR_BRAKE_STEP : MOTOR_ACCEL_STEP;
  return current < target ? Math.min(current + step, target) : current > target ? Math.max(current - step, target) : current;
}
const cmPerS = pwm => Math.abs(pwm) < 80 ? 0 : pwm * 0.2;

let failed = 0;
const check = (label, ok, detail) => { if (!ok) failed++; console.log(`${ok ? "PASS" : "FAIL"}  ${label}\n      ${detail}`); };

// Static responses
check("Table: lost / no echo / too close / at target / approaching",
  followSpeed(999) === 0 && followSpeed(150) === 0 && followSpeed(20) < 0 && followSpeed(50) === 0 &&
  followSpeed(80) > 0 && followSpeed(120) === FOLLOW_MAX_SPEED,
  [999, 150, 120, 80, 51, 50, 40, 20].map(d => `${d}cm->${followSpeed(d)}`).join("  "));

// car1(t) returns Car 1's PWM, or null when Car 1 has turned out of the beam
function chase(label, car1, ms, startGap, expect) {
  let gap = startGap, applied = 0, minGap = Infinity, maxGap = 0, lostAt = null;
  for (let t = 0; t < ms; t += 20) {
    const c1 = car1(t);
    const seen = c1 !== null && gap <= 400 ? Math.round(gap) : 999;
    applied = ramp(applied, followSpeed(seen));
    gap += (cmPerS(c1 ?? 0) - cmPerS(applied)) * 0.02;
    if (c1 !== null) { minGap = Math.min(minGap, gap); maxGap = Math.max(maxGap, gap); }
    else lostAt ??= t;
  }
  const r = { gap, minGap, maxGap, applied };
  check(label, expect(r), `gap now ${gap.toFixed(0)} cm (min ${minGap.toFixed(0)}, max ${maxGap.toFixed(0)}), Car 3 PWM ${applied}`);
}

chase("Car 1 drives away at explore speed: Car 3 keeps up without closing in", () => EXPLORE_SPEED, 20000, 60,
  r => r.minGap >= FOLLOW_TOO_CLOSE_CM && r.maxGap <= FOLLOW_LOST_CM - 15);
chase("...even if Car 3's motors are 10% weaker than Car 1's", t => EXPLORE_SPEED * 1.1, 20000, 60,
  r => r.maxGap <= FOLLOW_LOST_CM - 5);
chase("Car 1 stops: Car 3 stops near the target gap", t => t < 5000 ? EXPLORE_SPEED : 0, 15000, 60,
  r => r.applied === 0 && r.gap >= FOLLOW_TOO_CLOSE_CM && r.gap <= FOLLOW_TARGET_CM + 15);
chase("Car 1 stops and reverses (obstacle recovery): Car 3 never gets too close",
  t => t < 5000 ? EXPLORE_SPEED : t < 10000 ? 0 : t < 10350 ? -AVOIDANCE_REVERSE_SPEED : 0, 15000, 60,
  r => r.minGap >= FOLLOW_TOO_CLOSE_CM - 10);
chase("Car 1 turns out of the beam: Car 3 stops", t => t < 3000 ? EXPLORE_SPEED : null, 8000, 60,
  r => r.applied === 0);

console.log(failed ? `${failed} FAILED` : "ALL PASS");
process.exitCode = failed ? 1 : 0;
