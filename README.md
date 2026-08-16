# usv_radio_bridge

Vehicle-side RFD900x telemetry-radio bridge for the Navi USV project — the
successor to [`NaciGokhanBasaran/ARACPC`](https://github.com/NaciGokhanBasaran/ARACPC)'s
`navi_vehicle` package, rewritten in C++17/rclcpp and now integrated into the
onboard `jetson-ws` ROS 2 workspace.

## Why this repo exists

`ARACPC/navi_vehicle` (Python) is not owned by this account, so it can't be
updated in place. Its logic has been ported to a new C++ ROS 2 package,
**`usv_radio_bridge`**, plus a small companion interfaces package,
**`usv_gnc_msgs`**, and both now live as first-class packages inside
[`codeerzz/jetson-ws`](https://github.com/codeerzz/jetson-ws) on the
`feature/rfd-radio-bridge` branch — that is the canonical, actively developed
copy, built and tested as part of the full workspace.

This repo is a **standalone mirror** of those same two packages, published
here (under an account with full control) since the original `ARACPC` repo
isn't. If you're working on the onboard workspace, go to `jetson-ws`; this
repo exists so the vehicle-side radio bridge also has a home independent of
that workspace, matching the role `ARACPC` used to serve.

## What changed vs. `ARACPC/navi_vehicle`

- **Language**: Python → C++17/rclcpp — dedicated serial reader thread,
  priority-ordered TX queues, POSIX termios I/O, zlib-backed CRC32 (matches
  the original `zlib.crc32` output byte-for-byte).
- **Wire protocol**: same frame envelope (`0xAA55` magic, version, type,
  sequence, length, CRC32 trailer). `TELEMETRY` is redesigned and smaller
  (drops raw angular velocity/acceleration/GNSS-covariance, adds
  `battery_percent`/`gnss_fix_status`/`rtk_status`/`control_mode`). `CONTROL`
  and `KILLSWITCH` are fixed binary structs instead of JSON. Two new packet
  types, `AUTONOMY_COMMAND`/`AUTONOMY_STATUS`, let a ground-station GUI
  request and get a fast ack for an AUTO/MANUAL mode switch.
- **Autonomy integration**: on `AUTONOMY_COMMAND`, calls a new
  `gnc/set_control_mode` service (defined in `usv_gnc_msgs`, this repo) on
  `jetson-ws`'s existing `mode_mux_node` — the sole arbiter of vehicle mode.
  This bridge never decides mode itself, it only relays requests and reports
  the result.
- `publish_remote_control` now defaults to `false` (the Python reference
  defaulted to `true`, contradicting its own docs).

## Packages

- **`usv_gnc_msgs`** — `SetControlMode.srv`: request `{string mode}`
  ("AUTO"/"MANUAL"), response `{bool success, string message, string mode}`.
- **`usv_radio_bridge`** — the radio bridge node itself (`radio_bridge_node`).
  See `usv_radio_bridge/config/radio_bridge.yaml` for all parameters
  (serial port, baud rate, topic names, rates) and
  `usv_radio_bridge/launch/radio_bridge.launch.py` to run it standalone.

## Build

Requires ROS 2 Humble and `mavros_msgs`, `nmea_msgs` installed
(`sudo apt install ros-humble-mavros-msgs ros-humble-nmea-msgs`).

```bash
colcon build --packages-select usv_gnc_msgs usv_radio_bridge
```

In the full `jetson-ws` workspace these two packages are also required by
`mode_mux_node` (`usv_gnc_msgs`) and `usv_bringup`'s launch files
(`usv_radio_bridge`) — see that repo for the integrated system.
