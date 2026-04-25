# Rover — a Claude-powered Roomba

A modified iRobot Roomba 700 running ROS 2 Jazzy on a Raspberry Pi 5,
controlled in natural language via Telegram. The robot navigates with
Nav2 + slam_toolbox, reasons with Claude (Opus 4.7 / Sonnet 4.6), and
exposes a tool surface that lets the language model plan tasks, search
for objects with its camera, manage waypoints, recover from collisions,
and explore freely on a saved map.

> *"Rover, search the south side of the room and find my red book."*
>
> Rover queries the map, plans three waypoints, navigates between them,
> silently inspects each location with its camera, and replies:
> *"Found it — on the bookshelf next to the couch. Photo attached."*

---

## Capabilities at a glance

- **Natural-language control over Telegram.** Anything you'd ask a person
  on the floor — "go to the kitchen", "back up a couple feet", "what's
  around you?", "save this spot as the bedroom door". A Claude-powered
  reasoning node does the planning and tool dispatch.
- **Spatial self-awareness.** Robot reports its map-frame pose, nearest
  named location, lidar clearance in each direction, battery levels for
  both the Roomba pack and the Pi pack, and dock contact status.
- **Vision via Claude.** The forward USB camera grabs frames on demand;
  Claude analyzes them inline and replies in plain language. Photos are
  only pushed to the chat when the operator asked to see something —
  internal "checks" happen silently.
- **Waypoint memory.** Save the robot's current pose by name, optionally
  with auto-computed yaw to face a target. Distinguish positions ("I'm
  AT the thing") from viewpoints ("I want to LOOK AT the thing from here").
- **Free-form exploration.** Claude can describe the map, pick raw
  coordinate waypoints, and search areas you never explicitly named.
- **Bump recovery.** Below-lidar obstacles (shoes, cables, chair legs)
  trigger an emergency stop + reverse, get marked in the costmap, and
  the robot can retry around them.
- **Stop / cancel mid-action.** Any "stop", "cancel", "halt", "wait"
  message during a long-running task aborts whatever the robot is doing.

---

## Hardware

| Component | Details |
|-----------|---------|
| **Robot base** | iRobot Roomba 700 — vacuum components removed, cavity houses the Pi and wiring |
| **Compute** | Raspberry Pi 5 16GB, Ubuntu 24.04, ROS 2 Jazzy |
| **Lidar** | RPLidar C1 (CP2102N USB-UART) at 460800 baud |
| **IMU** | BNO08x at I2C `0x4B` (non-default address), mounted below the lidar |
| **Battery monitor** | INA219 at I2C `0x41`, shares bus with IMU |
| **Camera** | USB webcam at `/dev/video0` (frames captured via `ffmpeg`) |
| **Robot battery** | Stock Roomba NiMH pack — charged via the dock |
| **Pi battery** | 3S Li-ion pack (9.0V – 12.6V) — charged separately |

**Persistent serial paths** (stable across reboots):

```
Roomba:  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0
RPLidar: /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de422bbc516eef11a2f7e8c2c169b110-if00-port0
```

> **Why a Roomba?** The 700 series ships with a documented Open Interface
> (serial protocol), a usable bump bumper, two wheel encoders, a coverage
> of cliff IRs, and an integrated drive system. Removing the vacuum
> mechanics gives a cheap, well-instrumented mobile base. The OI's `Clean`
> and `Dock` commands refuse to run because the firmware notices the
> missing vacuum hardware — only low-level drive and sensor commands are
> used. See [`src/roomba/RoombaOpenInterface.pdf`](src/roomba/RoombaOpenInterface.pdf).

---

## System architecture

```
                        ┌──────────────────────┐
                        │ Telegram (your phone)│
                        └──────────┬───────────┘
                                   │ messages / photos
                                   ▼
                        ┌──────────────────────┐
                        │   telegram_node      │ HTTP long-polling
                        └──────────┬───────────┘
                                   │ /reasoning/input  (text)
                                   │ /reasoning/photo  (paths)
                                   │ /reasoning/response (text)
                                   ▼
                        ┌──────────────────────┐
                        │   reasoning_node     │ Claude Opus 4.7 / Sonnet 4.6
                        │   (Anthropic SDK)    │ tool calling + agentic loop
                        └──────────┬───────────┘
                                   │ /action/command  (JSON)
                                   │ /action/status   (JSON)
                                   ▼
                        ┌──────────────────────┐
                        │   action_executor    │ all robot primitives
                        └─────┬────────────┬───┘
                              │            │
                   /cmd_vel   │            │ NavigateToPose
                   /goal_pose │            │ Spin / BackUp / DriveOnHeading
                              ▼            ▼
              ┌──────────────────┐    ┌───────────────────────┐
              │   roomba_node    │◄───│   Nav2 stack          │
              │  (drives wheels) │    │  + slam_toolbox       │
              └─────────┬────────┘    │  (localization)       │
                        │             └───────────────────────┘
                /roomba/sensors                  ▲
                        │                        │ /scan from RPLidar
              ┌─────────▼────────┐               │ /odom from EKF
              │ bump_obstacle    │───/safety_obstacles
              │ (debounced bump  │
              │  + cliff guards) │
              └──────────────────┘
```

### Data flow during a typical command

1. You send *"go to the kitchen and tell me what you see"* via Telegram.
2. `telegram_node` publishes the text to `/reasoning/input`.
3. `reasoning_node`'s worker thread feeds the text to Claude with the
   robot's full tool catalog (~25 tools — see below).
4. Claude calls `goto(target="kitchen")`. The reasoning node dispatches
   `{"action": "goto", "target": "kitchen"}` to `/action/command` and
   blocks until `/action/status` reports `state="success"` (or failed).
5. `action_executor` checks if the robot is on the dock; if so, reverses
   off automatically before sending the Nav2 goal.
6. Nav2 plans, the robot drives. `bump_obstacle_node` watches sensors
   in case of a collision.
7. Once arrived, Claude calls `take_photo()`. `action_executor` runs
   `ffmpeg` on `/dev/video0`, writes `/tmp/roomba_view.jpg`, replies
   with the path. The reasoning node forwards the path to
   `/reasoning/photo` (telegram_node uploads to Telegram) AND reads
   the bytes back to Claude.
8. Claude's vision model interprets the image, generates a description,
   the reasoning node publishes the text to `/reasoning/response`,
   `telegram_node` sends it.

---

## ROS 2 packages

```
src/
├── roomba/                  ← main package, all custom nodes
├── batteries/               ← INA219 Python lib (Pi battery monitor)
└── rplidar_ros/             ← Slamtec driver, vendored
```

### `roomba` — custom nodes and tooling

```
src/roomba/
├── package.xml
├── setup.py
├── setup.cfg
├── RoombaOpenInterface.pdf  ← OI spec reference
├── roomba/                  ← Python nodes
│   ├── driver.py            ← low-level serial wrapper for Roomba OI
│   ├── roomba_node.py       ← /cmd_vel → wheels; publishes /wheel/odometry + /roomba/sensors
│   ├── imu_node.py          ← BNO08x I2C driver; publishes /imu/data
│   ├── roomba_teleop.py     ← keyboard teleop; live battery monitor
│   ├── action_executor.py   ← high-level robot primitives, single API surface
│   ├── bump_obstacle_node.py ← bump/cliff → costmap obstacles + emergency reverse
│   ├── reasoning_node.py    ← Claude API + tool calling
│   ├── telegram_node.py     ← Telegram Bot HTTP API ↔ ROS topics
│   ├── rick_roll.py         ← plays Never Gonna Give You Up on the Roomba speaker
│   └── stranger_things.py   ← plays the Stranger Things motif
├── launch/
│   ├── bringup.launch.py    ← shared: TFs, roomba, IMU, EKF, lidar, foxglove, action_executor, bumps
│   ├── mapping.launch.py    ← bringup + slam_toolbox in MAPPING mode (build a map)
│   ├── navigation.launch.py ← bringup + slam_toolbox in LOCALIZATION mode + Nav2 (day-to-day driving)
│   └── webcam.launch.py     ← standalone ustreamer for browser camera access
├── config/
│   ├── ekf.yaml                       ← robot_localization fusion params
│   ├── nav2_params.yaml               ← Nav2 stack config
│   ├── slam_params.yaml               ← slam_toolbox mapping mode
│   ├── slam_localization_params.yaml  ← slam_toolbox localization mode
│   └── locations.yaml                 ← named waypoints (rewritten by save_location tool)
├── maps/
│   ├── home.yaml + home.pgm           ← Nav2-readable occupancy grid
│   └── home.posegraph + home.data     ← slam_toolbox serialized graph (resume / loc mode)
└── scripts/
    └── save_map.sh           ← saves both the .yaml/.pgm and the posegraph in one go
```

---

## Installation

These steps assume a fresh Pi 5 with Ubuntu 24.04. Most of the heavy
lifting is ROS 2 Jazzy; the Claude integration adds two Python packages.

### 1. ROS 2 Jazzy

Follow the official [ROS 2 Jazzy install on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Install at minimum: `ros-jazzy-ros-base`, `ros-jazzy-nav2-bringup`,
`ros-jazzy-slam-toolbox`, `ros-jazzy-foxglove-bridge`,
`ros-jazzy-robot-localization`, `ros-jazzy-rplidar-ros`.

### 2. System packages

```bash
sudo apt install ffmpeg python3-pip python3-yaml
```

`ffmpeg` is used by the `take_photo` action to grab frames from
`/dev/video0`.

### 3. Python packages on the system Python

ROS 2 in Jazzy uses the **system Python** (`/usr/bin/python3`). All
non-ROS Python dependencies must be installed there:

```bash
sudo /usr/bin/python3 -m pip install --break-system-packages \
    anthropic requests adafruit-circuitpython-bno08x adafruit-circuitpython-busio
```

| Package | Used by |
|---|---|
| `anthropic` | `reasoning_node` (Claude API) |
| `requests` | `telegram_node` (Telegram Bot HTTP API) |
| `adafruit-circuitpython-bno08x` | `imu_node` |
| `adafruit-circuitpython-busio` | `imu_node` (I2C transport) |

### 4. Clone and build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone <this-repo-url> .   # or however you pulled it
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` is the right default — code changes in `src/` are
picked up after a node restart, no rebuild needed.

> **Important:** YAML and map files are *copied*, not symlinked, by
> colcon. After editing `nav2_params.yaml`, `slam_params.yaml`, or
> any file under `config/` or `maps/`, run
> `colcon build --packages-select roomba --symlink-install` to push
> the changes into `install/`.

### 5. Configure the Telegram bot (if you want chat control)

1. Message **@BotFather** in Telegram. Send `/newbot`, follow prompts,
   get a token like `1234567890:ABCdef...`.
2. Message **@userinfobot** to find your numeric user ID.
3. Append the following to `~/.bashrc` (filling in real values):

   ```bash
   # Rover (Claude-powered Roomba)
   export ANTHROPIC_API_KEY=sk-ant-...
   export TELEGRAM_BOT_TOKEN=1234567890:ABCdef...
   export TELEGRAM_ALLOWED_IDS=123456789       # your user ID; comma-separate for multiple
   export CLAUDE_MODEL=claude-sonnet-4-6        # default; swap to claude-opus-4-7 for harder reasoning
   ```

4. `source ~/.bashrc` (or open a new terminal).

If you don't want Telegram, you can skip steps 1–4 and drive the robot
by publishing to `/reasoning/input` directly with `ros2 topic pub`.

---

## Running

The system splits into three terminals (or `tmux` panes). Each terminal
needs ROS sourced; the env vars from `~/.bashrc` carry through.

### Terminal 1 — robot stack

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch roomba navigation.launch.py
```

This brings up: static TFs, the Roomba serial driver, the IMU driver,
robot_localization EKF (fuses wheel + IMU into `odom→base_link` TF),
RPLidar, slam_toolbox in localization mode (loads the saved map +
publishes the `map→odom` correction), Nav2 navigation stack
(planner, controller, behaviors, BT, collision_monitor),
`action_executor`, `bump_obstacle_node`, and `foxglove_bridge`.

### Terminal 2 — reasoning node

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run roomba reasoning_node
```

Watch the startup log: it tells you the active Claude model, whether
`effort` is on, and how many stale photos it cleaned up.

### Terminal 3 — Telegram bridge

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run roomba telegram_node
```

Look for `telegram_bridge ready (allowlist of N)`. Send your bot a test
message in Telegram — it should respond.

### Optional: keyboard teleop

```bash
ros2 run roomba roomba_teleop
```

Useful for driving the robot to spots you want to save as named
waypoints. Arrow keys / WASD; live readout of bumps and battery.

### Visualization (optional but recommended)

Install [Foxglove Studio](https://foxglove.dev/) on your laptop. Connect
to the Pi over WebSocket: `ws://<pi-ip>:8765`. Useful panels to add:

| Topic | Purpose |
|---|---|
| `/scan` | Lidar scan, projected through TF |
| `/map` | Occupancy grid from slam_toolbox |
| `/local_costmap/costmap` | Live local costmap (where Nav2 plans) |
| `/global_costmap/costmap` | Long-range costmap including safety marks |
| `/safety_obstacles` | Bump/cliff points the robot has hit |
| `/goal_pose` | Click-to-publish (sends a Nav2 goal) |

---

## Map management

### Build a map of a new area

```bash
# Terminal 1: SLAM in mapping mode
ros2 launch roomba mapping.launch.py

# Terminal 2: drive
ros2 run roomba roomba_teleop

# Terminal 3: when satisfied, save in BOTH formats
~/ros2_ws/src/roomba/scripts/save_map.sh
# (defaults to map name "home"; pass a different name to save_map.sh as $1)

# Rebuild so the new map lands in install/share/
cd ~/ros2_ws && colcon build --packages-select roomba --symlink-install
```

### Save a named location during a session

While the robot is in `navigation.launch.py` mode, drive it to a spot
with teleop, then talk to it via Telegram:

> *"Save this spot as kitchen."*

The reasoning node calls `save_location(name="kitchen")`. The current
`map → base_link` TF is recorded as a new entry in
[`config/locations.yaml`](src/roomba/config/locations.yaml). Saved
locations survive a `colcon build` because the YAML is written directly
to the `src/` tree, not just `install/`.

### Position vs viewpoint waypoints

Each saved location has a `type`:

- **`position`** (default) — the robot navigates to the saved (x, y)
  and stops in whatever heading it arrived with. *No* end-of-nav
  rotation. Best for "I'm AT the thing" waypoints (under the ping pong
  table, in the corner of the kitchen).
- **`viewpoint`** — the robot navigates to (x, y) **and** rotates to
  the saved yaw at the end so it's facing the target. Best for
  "I want to LOOK AT the thing from here" waypoints (a good observation
  pose for the bedroom door).

Why the distinction matters: end-of-nav rotation costs a couple seconds
and looks awkward when you don't need it. With most waypoints saved as
`position`, Claude can hop between them efficiently without a spin at
every stop.

**Saving a viewpoint** — the yaw is computed to face a target
automatically. You don't need to pre-orient the robot.

> *"Save this as bedroom_door_view, looking at the bedroom door."*

The reasoning node passes `facing_target="bedroom_door"` to
`save_location`, marks it as a viewpoint, and computes the yaw. When
you later say *"go look at the bedroom door"*, Claude calls
`goto("bedroom_door_view")` and the robot lands at the right pose
**already facing the door**.

**Saving a position** — same tool, no `facing_*`:

> *"Save this as kitchen."*

`type: position` is written to the YAML and goto won't bother orienting
on arrival.

**Backward compatibility** — entries saved before this distinction
existed (no `type` field in the YAML) are inferred from name suffix:
`_view` or `_spot` → viewpoint, anything else → position. Re-save
explicitly if you want to override.

### Deleting waypoints

> *"Forget the bedroom_door_view location."*

Or via the action executor directly:
`{"action": "delete_location", "name": "bedroom_door_view"}`.

---

## The reasoning system

`reasoning_node` is the only place the system talks to an LLM. It
maintains an in-memory conversation history with Claude, exposes the
robot's capabilities as ~25 tools, and runs a fully agentic loop:

```
user message in /reasoning/input
  └─► append to conversation
      └─► call client.messages.create(...)
          ├─► response is text only → publish to /reasoning/response, done
          └─► response includes tool_use blocks
              ├─► execute each tool (often blocks on /action/status)
              ├─► append tool_result blocks to conversation
              └─► loop
```

### Models

The default is **Claude Sonnet 4.6** for fast turn-around. Swap to
**Claude Opus 4.7** via `CLAUDE_MODEL` env var when you want maximum
reasoning quality (slower, more expensive). **Haiku 4.5** also works
for very simple commands — sub-second responses. Adaptive thinking is
on for all models that support it; the `effort` parameter is set to
`medium` on Sonnet/Opus.

### Tool catalog

**Self-awareness — fast snapshots, no photos:**

| Tool | Returns |
|---|---|
| `get_state` | Pose + nearest location + lidar clearance F/L/R/B + batteries + dock status — the "ground yourself" call before planning |
| `where_am_i` | Pose + nearest named location |
| `battery_status` | Both Roomba and Pi battery levels, voltage, charging state |
| `describe_map` | Map bounds, resolution, current pose, all named locations |
| `list_locations` | Just the location names |

**Navigation — high-level:**

| Tool | What |
|---|---|
| `goto(target)` | Navigate to a named location. Auto-undocks if currently on the charger. |
| `goto_pose(x, y, yaw_deg)` | Navigate to arbitrary map coordinates (free-form exploration). |
| `look_around` | Spin 360° in place (no captures). |

**Navigation — fine-grained:**

| Tool | What |
|---|---|
| `move_forward(distance_m)` | Drive forward a precise distance (Nav2 DriveOnHeading; collision-checked). |
| `move_backward(distance_m)` | Drive backward (Nav2 BackUp). |
| `rotate(angle_deg)` | Rotate in place by signed angle. + = CCW/left. |
| `face(target / x,y / heading_deg)` | Rotate to face a named location, raw coords, or an absolute map heading. |

**Perception:**

| Tool | What |
|---|---|
| `take_photo(share=true)` | Single frame from the camera. `share=false` for silent internal checks. |
| `scan_surroundings(num_photos=6, share=true)` | Spin and capture N evenly-spaced photos, then return to original heading. Atomic — can't stop early. Use for "describe the room" / "tell me what's around". For SEARCH ("find X"), Claude uses a manual loop of `take_photo` + `rotate` instead so it can stop on first hit. |

**Waypoints:**

| Tool | What |
|---|---|
| `save_location(name, facing_target?, facing_x?, facing_y?)` | Save current x/y. Without `facing_*` → POSITION waypoint (current heading saved but goto won't enforce it). With `facing_*` → VIEWPOINT waypoint (yaw computed to face the target; goto will rotate to face it on arrival). |
| `delete_location(name)` | Remove a saved waypoint. |

**Dock:**

| Tool | What |
|---|---|
| `dock` | Disables bump sensors for 60 seconds — operator manually drives the robot onto the dock during this window. |
| `undock` | Drives backward off the dock for ~4s. Auto-invoked by `goto`/`goto_pose` if currently docked. |

**Control:**

| Tool | What |
|---|---|
| `cancel` | Abort the current async action. Also fires automatically when the operator's message starts with "stop"/"cancel"/"halt"/"wait"/"pause"/"abort". |

### System prompt highlights

The full prompt lives in `reasoning_node.py` near the top. Key
ideas:

- The robot's persona is **Rover** — direct, slightly dry, first
  person, no emoji unless the operator uses them first.
- **Ground in real sensor data** before planning. Call `get_state`
  first when there's any spatial/safety question.
- **Silent perception** — only push photos to chat when the operator
  asked to see something. During a search, look at frames internally
  and only surface the one with the answer.
- **Bump recovery** — when a motion tool returns `canceled — caused
  by bump_*`, take a silent photo, identify which side of the frame
  the obstacle is on, plan an explicit escape (rotate AWAY from the
  obstacle ~30-45° → step sideways 0.3-0.5m), THEN retry the goto.
  The costmap mark from the bump isn't reliable enough to re-plan
  around, so plain retry tends to hit the same thing — the explicit
  escape gets the robot lateral clearance first. One retry max.
- **Search-from-here pattern** — for "find X" requests at the
  current position, Claude drives a manual scan loop with
  `take_photo` + `rotate` so it can stop the moment it spots the
  target, rather than committing to all six frames of
  `scan_surroundings`.
- **Exploration pattern** — when the user asks to search across an
  area, describe_map → pick 3-5 waypoints → at each, run the
  search-from-here pattern → only surface photos when the target
  is found.

---

## Direct API (no Claude in the loop)

You can drive the system directly by publishing to `/action/command`
with JSON bodies. Useful for testing, scripting, or when you want
deterministic behavior.

```bash
# List saved locations
ros2 topic pub --once /action/command std_msgs/String \
  '{data: "{\"action\": \"list\"}"}'

# Go somewhere
ros2 topic pub --once /action/command std_msgs/String \
  '{data: "{\"action\": \"goto\", \"target\": \"office\"}"}'

# Save a viewpoint
ros2 topic pub --once /action/command std_msgs/String \
  '{data: "{\"action\": \"save_location\", \"name\": \"door_view\", \"facing_target\": \"bedroom_door\"}"}'

# Cancel everything
ros2 topic pub --once /action/command std_msgs/String \
  '{data: "{\"action\": \"cancel\"}"}'
```

Watch the responses on `/action/status`:

```bash
ros2 topic echo /action/status
```

---

## Calibration notes

- **Effective wheel diameter = 0.0431m** (physical is ~72mm). The
  effective value bakes in tire compression and slip on this surface
  (plush basement carpet). On hardwood the robot will *over-shoot*
  distances — recalibrate if you change surfaces.
- **Wheelbase = 0.237m** for cmd_vel → per-wheel-velocity conversion.
  Yaw is **not** integrated from wheels (carpet slips heavily during
  in-place turns); EKF takes yaw from the IMU exclusively.
- **IMU axes are aligned with `base_link`** — the static TF is
  translation only, no rotation.
- **Lidar TF has yaw = π** — the RPLidar is mounted physically rotated
  180°. The `bump_obstacle_node` lidar summary accounts for this when
  bucketing scan points into front/left/back/right.
- **Wheel speed deadband = 30 mm/s.** Below this the Roomba motors
  stall on plush carpet. The driver clamps non-zero wheel velocities
  up to the deadband, preserving sign — Nav2's slow goal-orientation
  commands now actually complete.
- **Nav2 progress_checker = 0.3m / 6s.** A goto aborts if the robot
  can't travel 0.3m within 6s (tight by Nav2 default standards).
  Stops Nav2 from grinding through Spin/BackUp/Wait recovery loops
  when it's stuck or oscillating; the cancellation surfaces to
  `reasoning_node` and Claude plans an explicit escape route. Loosen
  in `nav2_params.yaml > controller_server > progress_checker` if
  you find legitimate goals being aborted.

---

## Gotchas (read once, save yourself an hour later)

- **Stale map / config in `install/`.** `colcon` *copies* YAML and map
  files into `install/share/` — `--symlink-install` does NOT symlink
  data files, only Python code. After editing anything in `config/`
  or saving a new map: `colcon build --packages-select roomba
  --symlink-install`. Otherwise Nav2 keeps reading the old version.
- **`save_map` ≠ `serialize_map`.** `save_map` writes `.pgm` + `.yaml`
  for Nav2's static layer. `serialize_map` writes `.data` + `.posegraph`
  for slam_toolbox's resume/localization. The `save_map.sh` helper runs
  both — use it.
- **Stale `~/install`.** If you ever ran `colcon build` from `~`
  instead of `~/ros2_ws`, you'll have a phantom `/home/ubuntu/install/`
  that ROS may still source. `rm -rf ~/install ~/build ~/log` and
  rebuild from the workspace root.
- **Two Python venvs.** There's `~/venv` (activated by `.bashrc`) and
  `~/ros2_ws/venv` (probably abandoned). ROS 2 ignores them and uses
  `/usr/bin/python3`. All Python deps for the project must be on the
  system Python — install with `sudo /usr/bin/python3 -m pip install
  --break-system-packages <pkg>`.
- **BNO08x at 0x4B, not 0x4A.** Hardcoded in `imu_node.py`. Don't
  remove the explicit address.
- **Photos in `/tmp` are session-scoped.** `reasoning_node` wipes
  `/tmp/roomba_*.jpg` at startup so a previous run's images don't
  leak into a new conversation.

---

## Troubleshooting

### Telegram message has no effect

1. Open another terminal, `ros2 topic echo /action/status`.
2. From Telegram, send *"list your locations"*.
3. Expected: a status message appears within a couple seconds.
4. If nothing appears: check that `reasoning_node` and `telegram_node`
   are running and have the env vars (`echo $ANTHROPIC_API_KEY`).

### Robot doesn't move when nav goals are sent

1. Is `navigation.launch.py` running (and not `mapping.launch.py`)?
2. `ros2 topic info /action/status -v` should show one publisher.
3. `ros2 run tf2_ros tf2_echo map base_link` — does it return non-zero
   coordinates? If TF lookup fails, slam_toolbox isn't localized yet.
   Wait a few seconds after launch; if still failing, check that
   `/scan` is publishing.

### Robot drifts off the walls during navigation

If lidar matches walls when stationary but drifts during long drives,
the issue is usually **odometry calibration on the current surface**.
Run a 2m straight-line tape-measure test: command a `move_forward(2.0)`
and measure actual distance. Adjust the `wheel_diameter` constant in
`roomba_node.py` proportionally.

### Auto-undock fires when not on the dock

Some modified Roombas false-fire the home-base bit on packet 34. If
this becomes a problem, gate the auto-undock on a stronger signal
(e.g., charging_state in {1,2,3}) — see `_do_goto` in `action_executor.py`.

### Carpet transitions trigger bump emergency

Tune `bump_debounce_seconds` (default 1.5s):

```bash
ros2 param set /bump_obstacle bump_debounce_seconds 2.5
```

If your environment has no real cliffs, leave `cliff_detection_enabled`
at `False` (the default) — the cliff IRs false-fire during pitch-up
events on uneven floors.

---

## File-by-file index

### Nodes (`src/roomba/roomba/`)

- **`driver.py`** — Pure serial wrapper around the Roomba Open Interface.
  Drive commands, song playback, sensor stream parsing. Imports
  `serial`, no ROS dependencies.
- **`roomba_node.py`** — ROS bridge for the driver. Subscribes to
  `/cmd_vel`, publishes `/wheel/odometry` and `/roomba/sensors`. Owns
  the wheel-speed deadband. Does NOT publish `odom→base_link` TF —
  the EKF does.
- **`imu_node.py`** — BNO08x I2C driver. Publishes
  `/imu/data` (sensor_msgs/Imu) at ~30 Hz with absolute orientation
  and gyro readings.
- **`roomba_teleop.py`** — Standalone keyboard teleop. Useful for
  driving the robot to spots before saving them as named waypoints.
  Live battery readout in the terminal.
- **`action_executor.py`** — The single API surface for everything
  the robot can do. Subscribes to `/action/command`, executes commands,
  publishes `/action/status`. Handles all the high-level primitives
  (goto, goto_pose, save_location, take_photo, scan_surroundings, ...)
  plus the ones that wrap Nav2 actions (NavigateToPose / Spin / BackUp
  / DriveOnHeading). Owns the auto-undock logic, the panoramic scan
  worker, and the `/scan`-based lidar summary.
- **`bump_obstacle_node.py`** — Bump and cliff sensor handling.
  Debounces bumps (carpet transitions cause sustained false bumps),
  optionally disables cliff sensors (false-fire on uneven floors),
  publishes a `/safety_obstacles` PointCloud2 to a dedicated Nav2
  costmap layer, cancels active nav goals, and emits
  `/bump/interrupt_event` so the reasoning layer can recover.
- **`reasoning_node.py`** — The Claude integration. System prompt,
  full tool catalog, agentic loop, conversation memory, queue + ACK,
  stop-keyword detection, photo forwarding to Telegram.
- **`telegram_node.py`** — HTTP long-polling bridge. Translates
  Telegram messages to `/reasoning/input`, forwards
  `/reasoning/response` text and `/reasoning/photo` paths back to
  the chat via Telegram's `sendMessage` and `sendPhoto`.
- **`rick_roll.py`** — Loads "Never Gonna Give You Up" into the
  Roomba's note buffer and plays it through the onboard speaker.
  Standalone script (not a ROS node). Don't run while
  `roomba_node` holds the serial port.
- **`stranger_things.py`** — Same idea, the Stranger Things motif.

### Configs (`src/roomba/config/`)

- **`ekf.yaml`** — `robot_localization` EKF. Trusts wheel forward
  velocity and IMU heading/yaw rate; ignores wheel-derived yaw.
- **`nav2_params.yaml`** — Nav2 stack: BT navigator, planner (NavFn),
  controller (DWB), behaviors (Spin, BackUp, DriveOnHeading, Wait),
  collision_monitor, costmaps with two obstacle layers (one for lidar,
  one dedicated to bump/cliff marks that scan raytracing can't clear).
- **`slam_params.yaml`** — slam_toolbox in mapping mode (used by
  `mapping.launch.py`).
- **`slam_localization_params.yaml`** — slam_toolbox in localization
  mode (used by `navigation.launch.py`). Loads the saved posegraph
  from `install/share/.../maps/home`.
- **`locations.yaml`** — Named waypoints. Rewritten programmatically
  by `save_location` and `delete_location` tools. Hand-editable.

### Launch files (`src/roomba/launch/`)

- **`bringup.launch.py`** — Shared base: static TFs (laser, IMU,
  camera links), `roomba_node`, `imu_node`, EKF, RPLidar driver,
  `foxglove_bridge`, `bump_obstacle_node`, `action_executor`.
- **`navigation.launch.py`** — Day-to-day: `bringup` + slam_toolbox
  in localization mode + Nav2 navigation stack (planner, controller,
  behaviors, BT, collision_monitor). NOT `nav2_bringup`'s default
  `bringup_launch.py` — that includes AMCL + map_server which we
  replace with slam_toolbox.
- **`mapping.launch.py`** — Map building: `bringup` + slam_toolbox
  in mapping mode.
- **`webcam.launch.py`** — Standalone `ustreamer` for browser-based
  camera viewing. Not used by `take_photo` (which uses ffmpeg
  directly).

### Other

- **`scripts/save_map.sh`** — Runs both `save_map` and `serialize_map`
  via slam_toolbox services. Pass an optional name (default `home`).
- **`maps/`** — Saved maps. `home.pgm` + `home.yaml` for Nav2 static
  layer, `home.posegraph` + `home.data` for slam_toolbox.
- **`RoombaOpenInterface.pdf`** — iRobot Open Interface spec, kept
  for offline reference.

---

## Roadmap / future work

- **Map extension during exploration sessions.** Currently the system
  is locked to localization-only against a saved map — driving into a
  truly new room can degrade localization. A "lifelong mapping" mode
  that loads the existing graph but allows extensions would unlock
  freeform new-territory exploration.
- **Persistent object memory.** A `remember(observation)` /
  `recall(query)` tool pair backed by a JSON file would let Claude
  build up a spatial mental model across sessions ("the blue chair is
  in the southwest corner of the room").
- **Streaming responses to Telegram.** Currently the user sees Claude's
  reply only after the full agentic loop completes. Streaming partial
  responses as Claude produces them would tighten the demo rhythm.
- **Voice input.** Telegram voice messages → Whisper transcription →
  `/reasoning/input` would make demos feel more like real
  conversation.
- **Custom IR docking.** Roomba's stock dock command refuses to run
  on this hardware (vacuum components missing); a custom routine using
  packets 17 / 52 / 53 (the IR receivers) is a natural project to
  pick up later. Today the robot disables bump sensors for 60 seconds
  and lets the operator teleop onto the dock.
- **Compressed image transport.** The current camera path uses
  `ffmpeg` to grab single frames. A proper `image_transport`
  pipeline with compressed image topics would enable continuous
  vision streaming if needed for object detection.

---

## Acknowledgements

- iRobot for the Open Interface and tolerant hardware.
- The Nav2 and slam_toolbox communities — both stacks "just worked"
  with relatively little tuning.
- Anthropic for Claude. The reasoning quality on Sonnet 4.6 alone
  carries most of the demo.
- Foxglove for the visualization tooling — debugging this without a
  good 3D viewer would have been miserable on a headless Pi.
