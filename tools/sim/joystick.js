// Runs the dashboard's joystick functions, extracted verbatim from src/main.cpp.
// Usage: node tools/sim/joystick.js [src/main.cpp]
const src = require("fs").readFileSync(process.argv[2] || "src/main.cpp", "utf8").split("\r").join("");
const grab = n => {
  const i = src.indexOf("function " + n + "(");
  return src.slice(i, src.indexOf("\n    }\n", i) + 6);
};

let stickState = { active: false, x: 0, y: 0 }, keys = new Set(), speed = { value: 200 }, out = [];
const stickEl = { style: {} };
const joystickEl = { getBoundingClientRect: () => ({ left: 0, top: 0, width: 200, height: 200 }) };
function sendCurrentAxes() { const { f, t } = currentAxes(); out = wheelSpeedsFromAxes(f, t); }

eval(grab("wheelSpeedsFromAxes") + grab("currentAxes") + grab("updateStick") + grab("setJoystickFromPointer") + `
  const knobY = () => Number(stickEl.style.transform.match(/,\\s*(-?[\\d.e-]+)px/)[1]);
  const knobX = () => Number(stickEl.style.transform.match(/\\((-?[\\d.e-]+)px/)[1]);
  const cases = [];
  const check = (label, ok, detail) => cases.push([label, ok, detail]);

  setJoystickFromPointer(100, 30);   // centre is (100,100)
  check("finger up    -> forward, knob up", out[0] > 0 && out[1] > 0 && knobY() < 0, out + " knobY " + knobY().toFixed(0));
  setJoystickFromPointer(100, 170);
  check("finger down  -> reverse, knob down", out[0] < 0 && out[1] < 0 && knobY() > 0, out + " knobY " + knobY().toFixed(0));
  setJoystickFromPointer(170, 100);
  check("finger right -> knob right", knobX() > 0, out + " knobX " + knobX().toFixed(0));
  setJoystickFromPointer(30, 100);
  const leftSpin = out;
  stickState.active = false; keys.clear(); keys.add("left"); sendCurrentAxes();
  check("finger left  == A key", leftSpin[0] === out[0] && leftSpin[1] === out[1], leftSpin + " vs A " + out);
  keys.clear(); keys.add("right"); sendCurrentAxes(); const dKey = out;
  stickState.active = true; setJoystickFromPointer(170, 100);
  check("finger right == D key", dKey[0] === out[0] && dKey[1] === out[1], out + " vs D " + dKey);
  stickState.active = false; keys.clear(); keys.add("up"); sendCurrentAxes(); const wKey = out;
  setJoystickFromPointer(100, 0);
  check("finger up    == W key", wKey[0] === out[0] && wKey[1] === out[1], out + " vs W " + wKey);

  let failed = 0;
  for (const [label, ok, detail] of cases) { if (!ok) failed++; console.log((ok ? "PASS  " : "FAIL  ") + label + "   [" + detail + "]"); }
  console.log(failed ? failed + " FAILED" : "ALL PASS");
  process.exitCode = failed ? 1 : 0;
`);
