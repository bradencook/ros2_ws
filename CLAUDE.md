# Roomba Autonomy Project

A modified Roomba 700 running ROS 2 Jazzy on a Raspberry Pi 5. The robot
navigates with Nav2 + slam_toolbox (localization mode), is controlled in
natural language via Telegram, and runs a Claude Opus/Sonnet agentic
loop that exposes ~25 robot tools.

This file is the technical reference. The user-facing showcase is
[`README.md`](README.md).

## Hardware

| Component | Details |
|-----------|---------|
| Robot base | iRobot Roomba 700 — vacuum components REMOVED; cavity houses Pi 5 and wiring |
| Compute | Raspberry Pi 5 (16GB), Ubuntu 24.04, ROS 2 Jazzy |
| Lidar | RPLidar C1 on USB (CP2102N), 460800 baud, mounted with **yaw=π** offset relative to base_link |
| IMU | BNO08x at I2C `0x4B` (not default 0x4A), mounted below lidar |
| Battery monitor | INA219 at I2C `0x41` (shares I2C bus with IMU) |
| Camera | USB webcam at `/dev/video0` (frames captured via `ffmpeg`; not a ROS topic) |
| Pi battery | 3S Li-ion, 9.0V empty / 12.6V full — read by INA219 |

**Persistent serial paths:**
- Roomba: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0`
- RPLidar: `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de422bbc516eef11a2f7e8c2c169b110-if00-port0`

**Roomba OI quirks:** The `Clean` and `Dock` opcodes don't work — firmware
detects missing vacuum components and refuses. Low-level drive/sensor
commands work fine. The "dock" tool we expose just disables bump sensing
for 60s so the operator teleops onto the charger.

## Package layout

```
src/
├── roomba/                  ← main package, all custom nodes
│   ├── roomba/              ← node code
│   │   ├── driver.py            low-level Roomba OI serial wrapper
│   │   ├── roomba_node.py       /cmd_vel → wheels; publishes /wheel/odometry + /roomba/sensors
│   │   ├── imu_node.py          BNO08x driver, publishes /imu/data
│   │   ├── roomba_teleop.py     keyboard teleop + battery monitoring
│   │   ├── action_executor.py   high-level robot primitives (single API surface)
│   │   ├── bump_obstacle_node.py bump/cliff → costmap obstacles + emergency reverse
│   │   ├── reasoning_node.py    Claude API + tool calling agentic loop
│   │   └── telegram_node.py     Telegram Bot HTTP API ↔ ROS topics
│   ├── launch/
│   │   ├── bringup.launch.py        TFs + roomba + IMU + EKF + lidar + foxglove + action_executor + bumps
│   │   ├── mapping.launch.py        bringup + slam_toolbox (mapping mode)
│   │   ├── navigation.launch.py     bringup + slam_toolbox (loc mode) + Nav2 navigation stack
│   │   └── webcam.launch.py         standalone ustreamer
│   ├── config/
│   │   ├── ekf.yaml                       EKF: wheel vx + IMU yaw → odom→base_link
│   │   ├── nav2_params.yaml               Nav2 (planner + DWB + behaviors + collision_monitor + 2 obstacle layers)
│   │   ├── slam_params.yaml               slam_toolbox mapping mode
│   │   ├── slam_localization_params.yaml  slam_toolbox localization mode (loaded by navigation.launch.py)
│   │   └── locations.yaml                 named waypoints (rewritten by save_location tool)
│   ├── maps/
│   │   ├── home.yaml + home.pgm           Nav2 static layer
│   │   └── home.posegraph + home.data     slam_toolbox graph
│   └── scripts/
│       └── save_map.sh           runs both save_map AND serialize_map services
├── batteries/               INA219 Python lib (used by teleop and action_executor)
└── rplidar_ros/             Slamtec driver, vendored
```

## Topic / data flow (navigation mode)

```
roomba_node              imu_node (BNO08x)
    │                        │
    │ /wheel/odometry        │ /imu/data
    ▼                        ▼
    └─────► ekf_node ◄───────┘     (robot_localization)
               │
               │ /odom (remapped from /odometry/filtered)
               │ odom→base_link TF
               ▼
        Nav2 stack ◄──── slam_toolbox (localization mode)
        (planner +              │
         controller +           │ map→odom TF
         behaviors +            │ /map (OccupancyGrid)
         collision_monitor)     ▼
               │
               │ cmd_vel_smoothed
               ▼
        collision_monitor ──── /cmd_vel ────► roomba_node
                                                │
                                /roomba/sensors │
                                                ▼
                                       bump_obstacle_node
                                                │
                                                ├─ /safety_obstacles (PointCloud2)
                                                └─ /bump/interrupt_event (String JSON)
```

**Key:** EKF owns the `odom→base_link` TF; `roomba_node` does NOT publish TF.
slam_toolbox owns `map→odom`. Wheel-derived yaw is NEVER trusted (carpet
slip); IMU is the only yaw source the EKF integrates.

## Reasoning + Telegram pipeline

```
Telegram chat ──► telegram_node ──► /reasoning/input ──► reasoning_node
                                                              │
                                                              │ Anthropic API
                                                              ▼
                                                          Claude (tool calling)
                                                              │
                                                              ▼
                              ┌────── /action/command (JSON)
                              │       /action/status (JSON)
                              ▼
                       action_executor ──► everything robot-side
                              │
                              ▼
                       /reasoning/photo ──► telegram_node ──► chat
                       /reasoning/response ─► telegram_node ──► chat
```

## Common workflows

### Day-to-day operation (3 terminals)

```bash
# Terminal 1 — robot stack
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch roomba navigation.launch.py

# Terminal 2 — reasoning / Claude
ros2 run roomba reasoning_node    # picks up env vars from ~/.bashrc

# Terminal 3 — Telegram bridge
ros2 run roomba telegram_node
```

Required env vars (in `~/.bashrc`):
- `ANTHROPIC_API_KEY`
- `TELEGRAM_BOT_TOKEN`
- `TELEGRAM_ALLOWED_IDS` (optional but recommended)
- `CLAUDE_MODEL` (optional, defaults to `claude-sonnet-4-6`)

### Map a new area

```bash
ros2 launch roomba mapping.launch.py    # SLAM in mapping mode
ros2 run roomba roomba_teleop           # drive
~/ros2_ws/src/roomba/scripts/save_map.sh
cd ~/ros2_ws && colcon build --packages-select roomba --symlink-install
```

### Send a direct nav goal (bypass Claude)

```bash
ros2 topic pub --once /action/command std_msgs/String \
  '{data: "{\"action\": \"goto\", \"target\": \"office\"}"}'
```

Or via the Nav2 action directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: { header: {frame_id: 'map'},
          pose: { position: {x: 1.0, y: 0.0, z: 0.0},
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} } }
}" --feedback
```

### Foxglove

`ws://<pi-ip>:8765`. The 3D panel's publish-on-click is wired to `/goal_pose`
and `/initialpose`. Subscribe to `/scan`, `/map`, `/local_costmap/costmap`,
`/global_costmap/costmap`, `/safety_obstacles`. **No RViz** — headless Pi.

Nav2 goals go on `/goal_pose`, NOT the Nav1 legacy `/move_base_simple/goal`.

## Calibration notes

- **Effective wheel_diameter = 0.0431 m** (physical is ~72mm). Calibrated
  for plush basement carpet — absorbs both tire compression and slip.
  Will overshoot distance on hardwood — recalibrate per surface.
- **Wheelbase = 0.237 m**. Used for cmd_vel → per-wheel-velocity
  conversion only. Not used for yaw (EKF ignores wheel-derived heading).
- **IMU axes aligned with base_link** — no rotation in the static TF,
  just translation.
- **Lidar yaw offset = π** in the base_link → laser static TF. The
  bump_obstacle_node lidar summary code accounts for this when bucketing
  scan points into front/left/back/right.
- **Wheel-speed deadband = 30 mm/s** in `roomba_node._apply_min_speed`.
  Below this the motors stall on plush carpet. Sign-preserving floor.
- **Progress checker = 0.3m / 6s** (`nav2_params.yaml > controller_server
  > progress_checker`). Tighter than the Nav2 default — aborts a goal
  fast when the robot's stuck or oscillating in a tight spot, instead
  of letting the BT cycle through Spin/BackUp/Wait recovery loops
  forever. The cancellation surfaces to reasoning_node and Claude
  decides what to do next (typically: take a silent photo, plan an
  escape, retry).

## Localization (slam_toolbox, NOT AMCL)

Earlier iterations of this stack used Nav2's bundled AMCL + map_server.
We replaced both with slam_toolbox in localization mode for the entire
nav stack:

- `navigation.launch.py` includes nav2_bringup's `navigation_launch.py`
  (just the navigation stack — no AMCL, no map_server)
- Plus our own `slam_toolbox` localization launch using
  `slam_localization_params.yaml`
- slam_toolbox publishes both `/map` (OccupancyGrid for the costmap's
  static layer) and the `map→odom` TF (the localization correction)

This gives much tighter localization on this robot's noisy odom than AMCL
did. The remaining AMCL-related parameters in `nav2_params.yaml` are
**dead code** — left in case we ever switch back, but Nav2 doesn't load
the `amcl:` block when AMCL isn't in the lifecycle.

## Bump / cliff handling

`bump_obstacle_node` subscribes to `/roomba/sensors` and:

- Debounces bumps for `bump_debounce_seconds` (default 1.5s) — carpet
  transitions cause sustained false bumps that we want to ignore.
- Cliff sensors are **disabled by default** (`cliff_detection_enabled=false`)
  because they false-fire when the front of the robot pitches up over
  surface transitions. Re-enable upstairs.
- On a real bump, publishes a PointCloud2 mark to `/safety_obstacles`
  (in base_link frame, at the time of contact). Nav2 has a dedicated
  `safety_obstacles` costmap layer with `clearing: false` so scan
  raytracing can't wipe these marks.
- Cancels the active Nav2 goal via the `cancel_goal` service.
- Reverses ~1s at 0.15 m/s.
- Publishes a `/bump/interrupt_event` (String JSON: hazard, contact_point,
  timestamp). reasoning_node picks this up and tags any nav action that
  was canceled within 3s as "caused by bump". The system prompt teaches
  Claude an EXPLICIT escape pattern (silent photo → identify obstacle
  side → rotate AWAY → step sideways → retry goto). We do NOT rely on
  the costmap mark from the bump for replanning — that path was flaky
  in practice (Nav2 would still try to plan straight through and hit
  the same thing). The escape route gets the robot real lateral
  clearance before retrying.

Marks have a configurable TTL (`mark_ttl_seconds`, default 60s). When a
mark expires, the node calls `/local_costmap/clear_entirely_local_costmap`
and `/global_costmap/clear_entirely_global_costmap`. The scan layer
repopulates immediately from the next /scan tick.

## Waypoint types — position vs viewpoint

Each entry in `locations.yaml` carries a `type` field:

- `position` (default) — `goto` sends the goal pose with `orientation`
  set to the BEARING from the robot's current pose to the target. The
  goal is "satisfied" with whatever heading the robot arrives with —
  no end-of-nav rotation. This avoids Nav2's RotateToGoal critic
  forcing a final spin to satisfy a yaw the operator doesn't care
  about.
- `viewpoint` — `goto` sends the saved yaw as the goal orientation.
  Nav2 reaches (x, y) AND aligns to that yaw at the end.

Set automatically by `save_location`:
- Without `facing_target` / `facing_x+y` → `type: position`
- With either → `type: viewpoint`

**Backward compatibility:** legacy entries lacking the `type` field
are inferred from name suffix. `_view` or `_spot` → viewpoint, else
position. Re-save explicitly to override.

The dispatch lives in `_dispatch_goto` (action_executor); the suffix
inference lives in `_load_locations`.

## Auto-undock

`_do_goto` and `_do_goto_pose` check the home_base bit on packet 34.
If set, they spawn a worker thread that:

1. Disables bump sensors
2. Reverses at 0.15 m/s for `UNDOCK_DURATION_S` (4s, ~60cm)
3. Re-enables bump sensors
4. Dispatches the original Nav2 goal

So Claude never has to remember to call `undock` first — `goto` handles
it transparently.

## Reasoning system

`reasoning_node` runs a fully agentic loop with Claude. Worker thread
pulls user messages off a queue, calls `client.messages.create()` with
the full tool catalog, executes any `tool_use` blocks (most of which
dispatch to `action_executor` and block on `/action/status`), and loops
until Claude produces a non-tool response.

**Models:** Sonnet 4.6 default (faster, cheaper, plenty smart for this
domain). Opus 4.7 via `CLAUDE_MODEL=claude-opus-4-7`. Adaptive thinking
is on; `output_config.effort` set to `medium` (omitted on Haiku).

**Tools** (~25): see the system prompt + tool definitions at the top of
`reasoning_node.py`. Categories:
- Self-awareness: `get_state`, `where_am_i`, `battery_status`,
  `describe_map`, `list_locations`
- High-level nav: `goto`, `goto_pose`, `look_around`
- Fine motion: `move_forward`, `move_backward`, `rotate`, `face`
- Perception: `take_photo(share)`, `scan_surroundings(num_photos, share)`
- Waypoints: `save_location(name, facing_target?, facing_x?, facing_y?)`,
  `delete_location`
- Dock: `dock`, `undock`
- Control: `cancel`

**Persona:** "Rover" — direct, slightly dry, first person. Detailed
guidance in the SYSTEM_PROMPT constant.

**Stop detection:** `_on_input` checks if a message starts with
stop/cancel/halt/wait/pause/abort/nevermind/etc. while the worker is
busy, and if so publishes `{"action":"cancel"}` directly to
`/action/command`. The currently-blocked tool returns canceled, the
agentic loop unwinds naturally, no multithreading inside Claude.

**Silent perception:** `take_photo` and `scan_surroundings` accept
`share=false` to capture without forwarding to Telegram. System prompt
tells Claude to use silent mode for internal checks (search-style
flows where only the *finding* warrants a photo to the chat).

**Photo cleanup:** On startup, `reasoning_node` deletes
`/tmp/roomba_view.jpg` and `/tmp/roomba_scan_*.jpg` so a previous
session's images don't leak into a new conversation.

## Gotchas

- **Stale config in `install/`.** YAML and map files are copied (not
  symlinked) into `install/share/` by colcon. After editing anything
  in `config/` or `maps/`, run `colcon build --packages-select roomba
  --symlink-install`. Otherwise Nav2 / slam_toolbox load the OLD version.
- **`save_map` ≠ `serialize_map`.** Different services, different output
  files. `save_map.sh` runs both — always use the script.
- **Stale `~/install`.** If you ever accidentally ran `colcon build` from
  `~`, `rm -rf ~/install ~/build ~/log` and rebuild from `~/ros2_ws`.
- **Two Python venvs.** `~/venv` and `~/ros2_ws/venv`. ROS uses neither —
  it uses `/usr/bin/python3`. All non-ROS deps must be installed there
  with `sudo /usr/bin/python3 -m pip install --break-system-packages …`.
- **collision_monitor needs polygons.** Empty list crashes Nav2 bringup.
  Current config uses `FootprintApproach` polygon tied to the robot
  footprint.
- **docking_server needs ≥1 plugin.** We have `simple_charging_dock`
  configured but no actual dock instances — just satisfies the init
  check. The Nav2 docking_server is not actually used; we have our own
  teleop-window dock flow.
- **BNO08x at 0x4B**, not 0x4A. Hardcoded.

## If something stops working

In rough order of likelihood:

1. **You forgot to rebuild after editing a YAML.** `colcon build
   --packages-select roomba --symlink-install`. Check
   `[map_io]: Loading yaml file:` in the launch logs to confirm which
   path is being read.
2. **slam_toolbox hasn't localized yet.** Wait ~5s after launch. If
   `tf2_echo map base_link` keeps failing, scan/IMU/odom is broken.
3. **Stale map files.** `ls -la src/roomba/maps/` vs `ls -la
   install/roomba/share/roomba/maps/` — timestamps should match.
4. **API key unset.** `echo $ANTHROPIC_API_KEY` in the reasoning_node
   terminal. `~/.bashrc` doesn't always source on fresh shells; open
   a new terminal explicitly.
5. **IMU locked up.** BNO08x occasionally needs a power cycle —
   restart the launch (or replug the I2C if persistent).
6. **Calibration drifted on a new surface.** Run a 2m straight-line
   tape-measure test, adjust `wheel_diameter` in `roomba_node.py`.
