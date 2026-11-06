# SWARM-X roles

Flash the same firmware to all three ESP32 cars, changing only `ROBOT_ID` in `src/main.cpp` before each build:

| ID | Car | Primary responsibility |
| --- | --- | --- |
| `1` | Explorer | Drives forward in **Explore** mode. At 35 cm it reverses, scans left and right, and takes a path with at least 60 cm clear. Reports its distance and whether it is blocked. |
| `2` | Observer | In **Map** mode, measures front, right, back and left (four 90° turns, ending at its starting heading) and reports the four distances. |
| `3` | Coordinator / Support | Hosts the Wi-Fi control page and sends mission commands. Shows Car 1 and Car 2 observations. Turns Car 1 back when it is blocked, and follows Car 1 at about 50 cm during Explore. |

Car 3 creates the visible operator Wi-Fi network **`SWARM-X`** and Cars 1 and
2 join it. Connect a phone or laptop to `SWARM-X` (password: `swarmx12345`),
then open `http://192.168.4.1` for the dashboard. Car 3 can also join your
home Wi-Fi: copy `include/wifi_credentials.example.h` to the ignored private
file `include/wifi_credentials.h` and fill in the router name and password.
The SWARM-X network remains available even if the home network is unavailable.
Cars 1 and 2 keep retrying until Car 3's network is up, so power-on order does
not matter.

## Mission controls

- **Explore**: Car 1 explores with local obstacle recovery. Car 3 follows
  whatever is directly ahead of it (Car 1) at about 50 cm and stops if it loses
  sight of it. Car 2 holds position.
  - If Car 1 reports blocked (front, left and right all closed), Car 3 sends it
    back the way it came (180° turn), then resumes Explore. After 2 turn-backs
    without 8 s of free exploring, the team pauses in **Coordinate** and the
    dashboard asks the operator to choose a mode.
- **Map**: Car 2 runs one surroundings scan, then holds. Press **Map** again to
  rescan. Cars 1 and 3 hold position. This is an observation checkpoint, not SLAM.
- **Coordinate**: all cars hold position for replanning.
- **Manual**, **Avoid**, and **Line**: calibration/assisted modes; the
  coordinator sends the same wheel commands to the whole team.
- **Idle**: emergency team stop.

Safety: Cars 1 and 2 stop within 0.8 s if they stop hearing Car 3. Car 3 switches
the team to Idle if the dashboard is closed for 2 s.

## Calibrate on the real cars

- `AVOIDANCE_TURN_90_MS`: the scan, turn-back and avoidance turns all assume
  this many milliseconds is a 90° turn.
- `FOLLOW_MIN_SPEED`: must be enough PWM to get Car 3 moving.
- Check that the **D** key turns the car right. If it turns left, the motor sides
  are swapped and "right"/"left" in the scan labels are mirrored.

## Limits

Each car has one front-facing ultrasonic sensor and no position estimate.
Car 2's scan directions are relative to Car 2's own heading, so they cannot be
used to steer Car 1, and the team cannot tell whether two cars are in the same
area. Both need position sensing (wheel encoders, IMU or external localization)
first; the telemetry protocol can be extended with those fields.

## Checks

The `tools/sim` scripts run the real functions from `src/main.cpp` in Node:

```
node tools/sim/avoid.js        # Car 1 obstacle recovery
node tools/sim/observer.js     # Car 2 scan
node tools/sim/coordinator.js  # Car 3 turn-back coordination
node tools/sim/follow.js       # Car 3 follow controller
node tools/sim/joystick.js     # dashboard joystick
node tools/sim/page.js         # dashboard script + display helpers
```
