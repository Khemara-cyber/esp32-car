// Syntax-checks the dashboard script embedded in src/main.cpp and runs its
// display helpers against sample /status data.
// Usage: node tools/sim/page.js [src/main.cpp]
const vm = require("vm");
const src = require("fs").readFileSync(process.argv[2] || "src/main.cpp", "utf8").split("\r").join("");
const script = src.slice(src.indexOf("<script>") + 8, src.indexOf("</script>"));
new vm.Script(script);   // throws on a syntax error
console.log("PASS  dashboard script parses");

const grab = n => {
  const i = script.indexOf("function " + n + "(");
  if (i < 0) return "";
  return script.slice(i, script.indexOf("\n    }\n", i) + 6);
};
const helpers = ["formatDistance", "showTeammate", "showScan", "showCoordination"].map(grab).join("\n");
const ctx = {};
vm.runInNewContext(helpers + "\nthis.api = { formatDistance, showTeammate," +
  " showScan: typeof showScan === 'function' ? showScan : null," +
  " showCoordination: typeof showCoordination === 'function' ? showCoordination : null };", ctx);

let failed = 0;
const check = (label, got, want) => {
  const ok = got === want;
  if (!ok) failed++;
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}: "${got}"${ok ? "" : ` (expected "${want}")`}`);
};
const el = () => ({ textContent: "", className: "" });

let e = el(); ctx.api.showTeammate(e, { online: true, distance: 120, blocked: false });
check("Car online, 120 cm", e.textContent, "120 cm");
e = el(); ctx.api.showTeammate(e, { online: true, distance: 999, blocked: false });
check("Car online, no echo", e.textContent, "open");
e = el(); ctx.api.showTeammate(e, { online: true, distance: 30, blocked: true });
check("Car online, blocked", e.textContent + "|" + e.className.trim(), "blocked|stat-value blocked");
e = el(); ctx.api.showTeammate(e, { online: false, distance: 120, blocked: false });
check("Car offline", e.textContent, "offline");

if (ctx.api.showScan) {
  const cells = [el(), el(), el(), el()], state = el();
  ctx.api.showScan(state, cells, { id: 3, readings: 2, cm: [120, 40, 0, 0] }, 60);
  check("Scan in progress", state.textContent + " | " + cells.map(c => c.textContent).join(" "),
    "scanning 2/4 | 120 cm 40 cm ... ...");
  ctx.api.showScan(state, cells, { id: 3, readings: 4, cm: [120, 40, 999, 60] }, 60);
  check("Scan complete", state.textContent + " | " + cells.map(c => c.textContent + ":" + c.className.split(" ").pop()).join(" "),
    "scan #3 | 120 cm:open 40 cm:blocked open:open 60 cm:open");
}

module.exports = { ctx, check, el, done: () => {
  console.log(failed ? `${failed} FAILED` : "ALL PASS");
  process.exitCode = failed ? 1 : 0;
}, fail: () => failed++ };
if (require.main === module) module.exports.done();
