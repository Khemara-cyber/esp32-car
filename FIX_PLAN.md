# SWARM-X Fix Plan

Findings come from the code review of commit `d3f3470`. Each fix is done and
verified on its own before the next one starts. Verification is either a
firmware build (`pio run`) or running the real code from `src/main.cpp` in
Node (`tools/sim/*.js`).

## Phase 1 — Bugs (make the current design work as intended)

- [x] **1. Firmware does not compile** — done: builds with and without the credentials file
  - Problem: `#include "wifi_credentials.h"` but that file does not exist.
  - Fix: create the git-ignored `include/wifi_credentials.h`, and fall back to
    "no home Wi-Fi" with `__has_include` so a fresh clone still builds.
  - Verify: `pio run` succeeds.

- [x] **2. Explorer locks up in open space** — done: `node tools/sim/avoid.js` 7/7 pass, build OK
  - Problem: no echo (nothing within ~4 m → `pulseIn` timeout → 999) is treated
    as "obstacle close", and an open side is scored 0, so the car locks.
  - Fix: treat no echo as maximum range (clear) once the sensor has produced at
    least one valid echo since boot. A sensor that has never answered still
    counts as blocked (fail-safe for a disconnected sensor).
  - Verify: simulation — open space keeps driving, wide-open side is chosen,
    never-connected sensor still stops.

- [x] **3. Coordinator's auto-pause is overwritten by the dashboard** — done: only `/mode` sets the mode, page follows `/status`; build + JS syntax OK
  - Problem: the page sends `m=<mode>` with every `/drive` (every 250 ms) and
    `/drive` resets `currentMode`, undoing the switch to Coordinate.
  - Fix: only `/mode` (button press) changes the mode; the page syncs its
    highlighted button from `/status` so it shows the auto-pause.
  - Verify: build + code trace.

- [x] **4. Joystick is inverted** — done: `node tools/sim/joystick.js` 6/6 pass (old code fails 3), build OK
  - Problem: dragging up reverses the car and draws the knob downward.
  - Fix: store the pointer's Y with the screen's sign so knob and drive follow
    the finger.
  - Verify: run the page's joystick functions — up = forward, knob moves up.

- [x] **5. Cars 1 and 2 freeze forever if Car 3 is late** — done: retries until joined, build OK
  - Problem: one 15 s join attempt, then an endless stop loop with no retry.
  - Fix: keep motors stopped and retry joining `SWARM-X` until it succeeds.
  - Verify: build.

- [x] **6. Missions keep running without an operator / stale blocked flag** — done: Idle after 2 s silence, blocked flag must be fresh, build OK.
  Also fixed: Car 3 compared `now` (read before `handleClient()`) with a newer
  `lastBrowserCommand`, which wrapped around and zeroed one loop's command.

- [x] **7. Cars 1 and 2 see false "coordinator lost" timeouts** (found while doing fix 6) — done: every elapsed-time check audited, build OK
  - Problem: `now` is read at the top of `loop()`, but the ESP-NOW callback
    (WiFi task, other core) can store a newer `receivedAt` during the 25 ms
    `pulseIn`. `now - receivedTime` then wraps to ~4 billion → treated as a
    timeout → mode forced to Idle for that loop, which also **resets Car 1's
    obstacle manoeuvre**. Estimated 29–83 % of packets trigger it.
  - Fix: measure the age with `millis()` taken after reading `receivedAt`
    (same for the dashboard's online/offline check).
  - Verify: uint32 arithmetic check + build.
  - Problem: Explore continues when the phone disconnects, and Car 1's blocked
    flag is used even when its telemetry is old.
  - Fix: switch to Idle after 2 s without contact from the page; ignore Car 1's
    blocked flag unless its telemetry is fresh.
  - Verify: build.

## Phase 2 — Missing SWARM-X behaviour (done)

New features. Verified by build + simulation of the real functions; all
timings (turn time, follow distances) still need calibrating on the real cars.

- [x] **8. Show team observations on the dashboard** — done: `node tools/sim/page.js` pass, build OK
  - Car 1 and Car 2 distance and blocked state, not just online/offline.
  - Verify: build + JS syntax check.

- [x] **9. Car 2 Observer scan** — done: `node tools/sim/observer.js` 8/8, `page.js` pass, build OK.
  The simulation caught a bug in the first version (each turn 30 ms short →
  330° instead of 360°), fixed. Pressing Map again rescans via a mode command id.
  - In Map mode Car 2 turns in four 90° steps, measures each direction, and
    broadcasts the four distances; Car 3 shows them as open/blocked.
  - Scans once per entry into Map mode, then holds (press Map again to rescan).
  - Verify: simulation of the scan state machine + build.

- [x] **10. Car 3 coordinates Car 1's recovery** (revised) — done: `node tools/sim/coordinator.js` 5/5, build OK.
  The simulation caught the first version turning 188° instead of 180°
  (one loop too long), fixed.
  - Original idea "use Car 2's scan to pick Car 1's direction" is not possible:
    Car 2's directions are relative to Car 2's own position and heading, and no
    car knows where it is. Using them would send Car 1 the wrong way.
  - Instead: when Car 1 reports blocked during Explore, Car 3 commands a
    180° turn-back (the way Car 1 came is known to be clear), then resumes
    Explore. After 2 recoveries without progress it pauses the team in
    Coordinate and the dashboard asks for the operator.
  - Verify: simulation of the coordinator logic + build.

- [x] **11. Car 3 support movement** — done: `node tools/sim/follow.js` 6/6, build OK.
  The simulation showed Car 3 could not close the gap at Car 1's speed
  (gap crept to 119 cm, lost at 120); its top speed is now Car 1's + 30.
  - In Explore mode Car 3 follows whatever is directly ahead of it (Car 1) at
    about 50 cm using its ultrasonic sensor; it backs off if too close and stops
    if it loses the target. Car 2 holds position.
  - Verify: simulation of the follow controller + build.

Limitation to keep in mind: with one front ultrasonic sensor per car and no
wheel encoders or IMU, no car knows its position. "Avoid moving into the same
area" and a real map need position sensing first.

## Final check (all items)

All six `tools/sim` suites pass and the firmware builds for `ROBOT_ID` 1, 2
and 3. `simulate_web.py` now serves `/mode` and a fake `/status`, so the full
dashboard can be previewed at http://localhost:8080 without the cars.

## Phase 3 — Test on the real cars (not done: needs the hardware)

- [ ] D key turns the car **right** (if left, motor sides are swapped)
- [ ] Calibrate `AVOIDANCE_TURN_90_MS` until a scan turn is 90°
- [ ] Car 2: Map mode turns 4 × 90° and ends facing where it started
- [ ] Car 1: Explore toward a wall — stops at ~35 cm, scans, takes the open side
- [ ] Car 1 in a dead end — Car 3 turns it back; after 2 tries the page asks for the operator
- [ ] Car 3 follows Car 1 at ~50 cm; raise `FOLLOW_MIN_SPEED` if it does not start moving
- [ ] Close the dashboard during Explore — all cars stop within ~2 s
- [ ] Power Cars 1/2 before Car 3 — they join once Car 3 is up

## Phase 4 — Problems found while testing on the cars

- [x] **12. Dashboard keeps showing "Connection problem"** — code done: all sims pass, builds for IDs 1–3. Real-car test still needed.
  - Cause (from the installed library source, `WebServer.cpp`/`WebServer.h`):
    `WebServer` serves one connection at a time and waits up to
    `HTTP_MAX_DATA_WAIT` = 5000 ms (not overridable) for a request on it.
    Chrome opens spare connections that send nothing (espressif/arduino-esp32
    issue 3652), so the server stalls; every `/drive` hits the page's 400 ms
    abort → "Connection problem", motors stop after 0.8 s, Idle after 2 s.
    Made worse by `/status` polls that start every 250 ms without waiting for
    the previous one (browser limit: 6 connections per host).
  - Fix: switch to ESP-IDF's `esp_http_server`, already built into the ESP32
    framework (no extra library): it serves several connections in parallel
    and drops the least recently used one when full. Handlers
    then run in another task, so they only hand values to `loop()` under a
    lock. Page: one `/status` at a time with a timeout, and show the error only
    after 2 failed commands in a row.
  - Verify: build all three IDs + page checks. Real test: drive for a few
    minutes from Chrome without the message appearing.
