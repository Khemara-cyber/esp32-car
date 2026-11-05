# SWARM-X roles

Flash the same firmware to all three ESP32 cars, changing only `ROBOT_ID` in `src/main.cpp` before each build:

| ID | Car | Primary responsibility |
| --- | --- | --- |
| `1` | Explorer | Drives forward in **Explore** mode, detects blocked paths, and reports its range observation. |
| `2` | Mapper / Observer | Holds position in **Map** mode, samples its surroundings, and reports range observations to the coordinator. |
| `3` | Coordinator / Support | Hosts the Wi-Fi control page, distributes mission commands, monitors Car 1 and Car 2 telemetry, and pauses exploration when the explorer reports that it is blocked. |

Car 3 creates the visible operator Wi-Fi network **`SWARM-X`** and Cars 1 and
2 join it. Connect a phone or laptop to `SWARM-X` (password: `swarmx12345`),
then open `http://192.168.4.1` for the dashboard. Car 3 can also join your
home Wi-Fi: copy `include/wifi_credentials.example.h` to the ignored private
file `include/wifi_credentials.h` and fill in the router name and password.
The SWARM-X network remains available even if the home network is unavailable.

## Mission controls

- **Explore**: only Car 1 advances. Its local obstacle recovery remains active; Cars 2 and 3 hold position.
- **Map**: all cars hold position while Car 2 continues publishing observations. This is a deliberate observation checkpoint, not SLAM.
- **Coordinate**: all cars hold position for replanning or support.
- **Manual**, **Avoid**, and **Line**: calibration/assisted modes; the coordinator distributes the requested wheel commands to the team.
- **Idle**: emergency team stop.

Each non-coordinator car stops within the control timeout if it stops receiving commands. The coordinator dashboard reports whether Car 1 and Car 2 have recently sent telemetry.

The current sensor set provides a front range observation only. A real spatial map needs a position estimate (wheel encoders, IMU, or external localization) and additional sensing; the telemetry protocol is intentionally structured so those fields can be added without changing the role model.
