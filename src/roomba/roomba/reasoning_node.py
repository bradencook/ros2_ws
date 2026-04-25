#!/usr/bin/env python3
"""Claude-powered reasoning layer.

Subscribes:  /reasoning/input   (std_msgs/String) — natural-language command
Publishes:   /reasoning/response (std_msgs/String) — text reply for the user
Publishes:   /action/command    (std_msgs/String) — JSON dispatch to action_executor
Subscribes:  /action/status     (std_msgs/String) — JSON status from action_executor

The node maintains a conversation with Claude (Opus 4.7) and uses tool calling
to invoke robot primitives. The API call runs in a worker thread so ROS
callbacks stay responsive while the model thinks.

Requires:
  pip install anthropic
  export ANTHROPIC_API_KEY=...
"""

import base64
import glob
import json
import math
import os
import queue
import threading
from typing import Any

import anthropic
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SYSTEM_PROMPT = """You are Rover, a modified iRobot Roomba 700 series running
ROS 2 Nav2 on a Raspberry Pi 5. You live in a carpeted basement. Your vacuum
components have been removed so the cavity houses your compute — you can't
vacuum, but you can drive, see (RPLidar for walls, a small forward USB
camera for colour), localize against a saved map (SLAM Toolbox), and plan
routes (Nav2). You're autonomous; treat the human as a friendly operator.

Voice: direct and a little dry. First person. No emoji unless the user uses
them first. Don't narrate what you're about to do before calling tools —
just call them. If a tool fails, say what failed and what you'd try next.

=== GROUND YOURSELF IN REAL SENSOR DATA ===
Conversation history goes stale fast — the robot moves, batteries drain,
obstacles appear. Before making any non-trivial decision, CHECK THE SENSORS.

Default workflow for motion/exploration/search:
  1. get_state — fast snapshot (pose, nearest location, lidar clearance,
     battery, dock). Use this FIRST when a request involves moving or
     reasoning about "where am I / is this safe / is this nearby".
  2. describe_map or list_locations if you need more spatial context.
  3. Plan the action based on what the sensors actually say.

Do NOT assume:
  - The robot is in the same place it was earlier in the conversation
  - A path is clear just because it was last time
  - The battery is fine just because it was fine a few turns ago

=== SILENT PERCEPTION ===
take_photo and scan_surroundings have a `share` parameter. By default
(share=true) the image(s) go to the operator's chat. Pass share=false when
you're using perception INTERNALLY for reasoning and the operator didn't
ask to see anything — e.g. during a silent search, during a "is the path
clear?" check, or when disambiguating locations. Only surface photos when
the operator's request implies they want to SEE something ("what do you
see", "take a photo", "show me the kitchen"). During an internal search,
describe findings in words — only push the photo of the found object, not
every frame along the way.

High-level navigation:
- goto: navigate to a named location on the map. If you're currently on
  the charging dock, goto handles undocking for you automatically — you do
  NOT need to call undock first.
- look_around: spin in place (full 360°) to look in all directions —
  nothing is captured. Use only when you want to reorient; for describing
  surroundings use scan_surroundings instead.
- save_location: save the robot's current position under a name. The
  WAYPOINT TYPE controls how goto behaves later:
    * POSITION (default) — the robot navigates to (x, y) and stops in
      whatever heading it arrived with (no end-of-nav rotation). Use for
      "I'm AT the thing" waypoints (under the ping pong table, on the
      dock). Call: save_location(name=...).
    * VIEWPOINT — the robot navigates to (x, y) AND rotates to face the
      saved target at the end. Use for "I want to LOOK AT the thing from
      here" waypoints. Call: save_location(name=..., facing_target=
      "<existing location>") and the yaw is auto-computed; the operator
      doesn't have to pre-orient the robot. Suffix the name with "_view"
      or "_spot" so its purpose is clear and so legacy entries without an
      explicit type get treated as viewpoints too.
  Goto only orients the robot at the end if the location is a viewpoint —
  this means Claude can hop between positions efficiently without an
  unnecessary spin at every stop.
- delete_location: remove a saved location when the operator asks to
  forget/remove/delete it.
- list_locations: list all named map locations.

Fine-grained motion (for small adjustments):
- move_forward: drive forward a specified distance (meters). Convert
  feet/inches yourself (1 foot ≈ 0.3m, 1 inch ≈ 0.025m).
- move_backward: drive backward a specified distance.
- rotate: rotate in place by a signed angle (degrees). Positive = left/CCW,
  negative = right/CW. Use for relative turns ("turn left 30°").
- face: rotate in place to face a named location, arbitrary (x, y) coords,
  OR an absolute map heading — WITHOUT driving there.
  - `target=<name>`: face a saved location.
  - `x`+`y`: face a map coordinate.
  - `heading_deg`: face an absolute map compass bearing. This is how you
    revisit a direction you saw in a prior scan — each scan photo is
    labelled with its map heading, so `face(heading_deg=<that heading>)`
    points the robot back at that photo's direction, even if the robot
    has moved/rotated since the scan.

Dock:
- dock: disables bump sensors for 60 seconds so the operator can manually
  drive the robot onto the charging dock. Announce when calling this.
- undock: drive backward off the dock.

Perception:
- take_photo: capture a single image from the front camera (current heading).
  Image is auto-sent to the operator's chat — do NOT say "here's the photo",
  just describe WHAT YOU SEE.
- scan_surroundings: spin and take 6 photos (every 60°), then return to the
  original heading. All photos auto-sent. Use for "what's around you",
  "search the room for X", "describe the space". Angle labels: 0° = the
  robot's heading at the START of the scan.

Self-awareness:
- where_am_i: returns current map-frame pose + nearest named location +
  distance. Use for "where are you?", "how close are you to X?", or to
  ground your own plan ("am I already near the kitchen?").
- battery_status: returns BOTH battery packs — the Roomba NiMH pack (drives
  the motors, chargeable via the dock) and the Pi 3S Li-ion pack (powers
  the compute, charged separately via USB-C/wall). Report both when asked
  "how's your battery?". If only one is mentioned by the user, use context
  to pick. Typical low-battery cutoffs: Roomba below 20% or Pi below 20%.

Free-form exploration:
- describe_map: map bounds, resolution, current pose, and all named
  locations with coordinates. Call this first when you need to reason
  about where to explore.
- goto_pose: navigate to arbitrary (x, y, yaw_deg) map coordinates.

Search-from-here pattern (manual scan loop) — when the operator wants
you to look around YOUR CURRENT POSITION for a specific target ("find my
keys here", "look for the red ball in this room"). DO NOT use
scan_surroundings for search — it captures all 6 photos before you can
react. Instead, drive the loop yourself so you can stop the moment you
find the target:
  1. take_photo(share=false). Look at the image.
  2. If you see the target → call take_photo(share=true) so the operator
     gets the photo, describe what you see, stop. Don't keep rotating.
  3. If not → rotate(angle_deg=60).
  4. Repeat steps 1-3. After 6 iterations you've covered 360°.
  5. If the full sweep finishes with no target, briefly say what you saw
     and ask if the operator wants to try a different location.

Exploration pattern (search across an AREA, not just from one spot) —
when the operator asks you to explore an unnamed area ("look on the west
side", "search the basement"):
  1. get_state to ground yourself (position, lidar clearance, battery).
  2. describe_map for the coordinate space and named anchors.
  3. Pick 3–5 candidate waypoints inside the map bounds, spaced enough
     to see different angles.
  4. At each waypoint: goto_pose → run the search-from-here pattern
     above (manual scan loop). Stop as soon as you find the target.
  5. Only SHARE the photo when you've actually found something. Don't
     flood the chat with every frame.
  6. If a goto_pose fails (Nav2 can't plan there), pick a different
     point — don't retry the same coordinates.

Control:
- cancel: abort the current action.

Bump recovery: if a motion tool returns 'canceled — caused by bump_*',
the robot physically hit something the lidar didn't see. It has already
reversed ~15cm. **Do NOT just retry the same goto.** The costmap mark
from the bump is unreliable and a plain retry usually hits the same
thing again or gets stuck. Plan an explicit escape route instead:

  1. take_photo with share=false (silent) to see what's in front of you.
     Don't push it to the chat unless the operator asked to see something.
  2. Look at the photo and decide where the obstacle is — LEFT side of
     frame, RIGHT side, or CENTER (mostly blocking forward).
  3. Plan a small escape sequence — typically 2-3 moves:
       - Obstacle on the LEFT  → rotate RIGHT, e.g. rotate(angle_deg=-35)
       - Obstacle on the RIGHT → rotate LEFT, e.g. rotate(angle_deg=35)
       - Obstacle in the CENTER → pick a side based on lidar clearance
         from get_state (rotate toward whichever side has more room)
       - Then move_forward 0.3-0.5m to put lateral distance between
         the robot and the obstacle.
  4. After the escape moves, call the ORIGINAL goto again. Nav2 plans
     a fresh path from the new pose, which usually clears the issue.
  5. If the retry also bumps OR you can't see what was hit, STOP and
     ask the operator. One retry max; do not loop.

You can also use get_state during the recovery to check lidar clearance
in each direction — that helps pick which way to rotate.

Nav2 oscillation: if a goto/goto_pose returns 'canceled' WITHOUT a
'caused by bump' annotation, Nav2's progress_checker decided the robot
wasn't making forward progress (stuck in a tight spot, oscillating in
place). Treat this similarly: take a silent photo, describe the area,
and either plan an escape or ask the operator. Do not blindly retry.

Reply concisely — one or two sentences is usually enough. When the operator
asks what you see, call take_photo or scan_surroundings and describe what's
visible. When they ask where you are, call where_am_i first.
"""


TOOLS = [
    {
        "name": "goto",
        "description": "Navigate to a named location on the map.",
        "input_schema": {
            "type": "object",
            "properties": {
                "target": {"type": "string", "description": "Name of the location (e.g. 'kitchen'). Call list_locations first if unsure."},
            },
            "required": ["target"],
        },
    },
    {
        "name": "look_around",
        "description": "Spin the robot 360 degrees in place without capturing images. Use when the user wants the robot to visually sweep but doesn't need you to see anything — e.g., to orient itself. For 'tell me what's around', use scan_surroundings instead.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "scan_surroundings",
        "description": "Rotate in place and capture N evenly-spaced photos (default 6, every 60°), then rotate back so the robot ends facing the same direction it started. All photos are returned to you at once with angle labels (relative AND absolute map heading). USE FOR: 'tell me what's around', 'describe this room' — when the operator wants the full panorama. DO NOT USE FOR SEARCH (e.g. 'find my keys') — this tool can't stop early. For search, use the manual scan loop with take_photo + rotate so you can stop the moment you spot the target.",
        "input_schema": {
            "type": "object",
            "properties": {
                "num_photos": {"type": "integer", "description": "Number of evenly-spaced photos (2-12). Default 6."},
                "share": {"type": "boolean", "description": "If true (default), also push each photo to the chat. Set false for silent panoramas."},
            },
        },
    },
    {
        "name": "move_forward",
        "description": "Drive the robot forward in a straight line by a specified distance in meters. Use for small positioning adjustments. Convert feet/inches to meters.",
        "input_schema": {
            "type": "object",
            "properties": {
                "distance_m": {"type": "number", "description": "Forward distance in meters (positive). E.g. 0.3 for a small step, 1.0 for a meter."},
            },
            "required": ["distance_m"],
        },
    },
    {
        "name": "move_backward",
        "description": "Drive the robot backward in a straight line by a specified distance in meters.",
        "input_schema": {
            "type": "object",
            "properties": {
                "distance_m": {"type": "number", "description": "Backward distance in meters (positive — do NOT negate, direction is implied)."},
            },
            "required": ["distance_m"],
        },
    },
    {
        "name": "rotate",
        "description": "Rotate the robot in place by a signed angle in degrees. Positive = left / counter-clockwise, negative = right / clockwise. E.g. 30 for a small left turn, -90 for a quarter turn right.",
        "input_schema": {
            "type": "object",
            "properties": {
                "angle_deg": {"type": "number", "description": "Rotation in degrees. Positive = left/CCW, negative = right/CW."},
            },
            "required": ["angle_deg"],
        },
    },
    {
        "name": "save_location",
        "description": "Save or overwrite a named waypoint. Three input modes:\n- (A) Current pose: pass just `name`. Records the robot's current x/y/yaw. Type = position.\n- (B) Current pose viewpoint: pass `name` + `facing_target` (or `facing_x`+`facing_y`). Records current x/y but yaw is computed to face the target. Type = viewpoint.\n- (C) Explicit pose: pass `name` + `x` + `y` (+ optional `yaw_deg`, `type`). Lets you register or edit a waypoint WITHOUT the robot being there. Useful for editing existing entries, migrating types, or entering coords picked off a map.\nUse `type` to override: 'position' (no end-of-nav rotation) or 'viewpoint' (orient to saved yaw on arrival). Default is 'viewpoint' if any facing_* is given, 'position' otherwise. Prefer underscored names; '_view' or '_spot' suffix is conventional for viewpoint entries.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Name for this location."},
                "facing_target": {"type": "string", "description": "Optional. Yaw is computed so the saved pose faces this named location."},
                "facing_x": {"type": "number", "description": "Optional, paired with facing_y. Yaw is computed to face this map coordinate."},
                "facing_y": {"type": "number", "description": "Optional, paired with facing_x."},
                "x": {"type": "number", "description": "Optional explicit map-frame X (meters). Pair with `y` to register a waypoint without the robot being there."},
                "y": {"type": "number", "description": "Optional explicit map-frame Y (meters). Pair with `x`."},
                "yaw_deg": {"type": "number", "description": "Optional explicit yaw in degrees (0° = +X axis, positive = CCW). Only used in explicit-pose mode and only when no facing_* is given."},
                "type": {"type": "string", "enum": ["position", "viewpoint"], "description": "Optional explicit type override. 'position' = no end-of-nav rotation. 'viewpoint' = orient to saved yaw at end of goto."},
            },
            "required": ["name"],
        },
    },
    {
        "name": "delete_location",
        "description": "Remove a named location from the map. Use when the operator says 'delete X', 'forget about X', 'remove X from the map'.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Name of the location to delete."},
            },
            "required": ["name"],
        },
    },
    {
        "name": "dock",
        "description": "Disable bump sensors for 60 seconds so the user can manually drive the robot onto the charging dock.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "undock",
        "description": "Drive the robot backward off the charging dock.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "cancel",
        "description": "Cancel the current navigation or action.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "list_locations",
        "description": "List all named locations saved on the robot's map.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "take_photo",
        "description": "Capture a single photo from the robot's front-facing camera (current heading). Returns the image inline to you. By default also sends it to the operator's Telegram chat; pass `share: false` to capture SILENTLY — for internal reasoning (e.g. checking for an object during search) without flooding the chat. The operator only sees photos when you explicitly share them.",
        "input_schema": {
            "type": "object",
            "properties": {
                "share": {"type": "boolean", "description": "If true (default), also send the photo to the operator's chat. Set false for silent checks."},
            },
        },
    },
    {
        "name": "where_am_i",
        "description": "Get the robot's current position: map-frame coordinates (x, y), heading in degrees, and the nearest named location with its distance. Use when the operator asks 'where are you?', 'are you near X?', or when you need spatial context before planning a move.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_state",
        "description": "Fast compact snapshot of the robot's current situation: pose, nearest named location, lidar clearance in each direction (front/left/right/back), batteries, dock status. Under 1 second, no photos, not sent to the chat. Call this FIRST before making navigation/motion decisions so you plan off real sensor data instead of the conversation's memory.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "battery_status",
        "description": "Get the battery state: charge percent, voltage, charging state, and whether you're on the dock. Use when asked about battery or whether you need to dock.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "face",
        "description": "Rotate in place to face a named location, specific (x, y) coordinates, or an absolute map heading — WITHOUT driving there. Use ONE of: `target` (name), `x`+`y` (coords), or `heading_deg` (absolute map yaw, 0° = +X axis, positive = CCW). heading_deg is how you revisit a direction you saw during a prior scan — photos from scan_surroundings are labeled with map headings.",
        "input_schema": {
            "type": "object",
            "properties": {
                "target": {"type": "string", "description": "Name of a saved location to face."},
                "x": {"type": "number", "description": "Map-frame X in meters (pair with y)."},
                "y": {"type": "number", "description": "Map-frame Y in meters (pair with x)."},
                "heading_deg": {"type": "number", "description": "Absolute map heading to face, in degrees. 0°=+X, 90°=+Y. Use after a scan: each scan photo's label includes its map heading."},
            },
        },
    },
    {
        "name": "describe_map",
        "description": "Get the map's bounds, resolution, the robot's current map-frame pose, and all named locations with their coordinates. Call this before free-form exploration so you understand the coordinate space.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "goto_pose",
        "description": "Navigate to arbitrary (x, y) map coordinates with an optional heading. For exploring unnamed spots during a search. Prefer named goto when a location name fits. Call describe_map first so your coordinates are sensible. Auto-undocks if on the dock.",
        "input_schema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "Map-frame X in meters."},
                "y": {"type": "number", "description": "Map-frame Y in meters."},
                "yaw_deg": {"type": "number", "description": "Heading to face at the destination, in degrees. 0 = +X, positive = CCW/left. Default 0."},
            },
            "required": ["x", "y"],
        },
    },
]

# Per-action timeouts for tools that block until the action_executor reports
# completion. Long enough to cover realistic robot behavior; an expired timer
# is reported back to Claude as a failure so it can decide what to do.
# Model override: CLAUDE_MODEL env var wins, otherwise Sonnet 4.6.
# Recommended: sonnet-4-6 for fast demo pacing, opus-4-7 for hard reasoning,
# haiku-4-5 for quick back-and-forth on simple commands.
CLAUDE_MODEL = os.environ.get('CLAUDE_MODEL', 'claude-sonnet-4-6')

# Effort isn't supported on Haiku — drop it when the user picks Haiku.
USE_EFFORT = not CLAUDE_MODEL.startswith('claude-haiku')


ACTION_TIMEOUTS = {
    'goto':           180.0,  # nav to a named location can take a while
    'look_around':     60.0,  # full 360° spin with recovery behaviors
    'move_forward':    45.0,  # DriveOnHeading — distance dependent
    'move_backward':   45.0,  # BackUp
    'rotate':          30.0,  # Spin with arbitrary angle
    'dock':            90.0,  # 60s teleop window + buffer
    'undock':          15.0,  # 4s reverse + buffer
    'list':             5.0,
    'save_location':    5.0,
    'take_photo':      15.0,  # ffmpeg capture
    'scan_surroundings': 60.0, # 6 photos × (0.5s settle + ffmpeg + ~1.7s turn)
    'where_am_i':       3.0,
    'battery_status':   3.0,
    'describe_map':     3.0,
    'goto_pose':      180.0,  # same as goto
    'face':            30.0,  # same as rotate
    'delete_location':  5.0,
    'get_state':        3.0,
}


class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')

        self.client = anthropic.Anthropic()
        self.messages: list[dict[str, Any]] = []

        self.input_sub = self.create_subscription(
            String, '/reasoning/input', self._on_input, 10
        )
        self.response_pub = self.create_publisher(String, '/reasoning/response', 10)
        # Paths to images we want forwarded to the user's chat. telegram_node
        # subscribes here and uploads via Telegram's sendPhoto.
        self.photo_pub = self.create_publisher(String, '/reasoning/photo', 10)
        self.command_pub = self.create_publisher(String, '/action/command', 10)
        self.status_sub = self.create_subscription(
            String, '/action/status', self._on_status, 10
        )
        self.bump_event_sub = self.create_subscription(
            String, '/bump/interrupt_event', self._on_bump_event, 10
        )

        # Synchronization between the ROS callback thread (which receives
        # /action/status) and the worker thread (which waits for specific
        # action responses).
        self._status_lock = threading.Lock()
        self._status_events: dict[str, threading.Event] = {}
        self._status_payloads: dict[str, dict] = {}

        # Most-recent bump interrupt, used to annotate nav actions that were
        # canceled as a direct result of a bump.
        self._bump_lock = threading.Lock()
        self._last_bump_ns: int = 0
        self._last_bump_info: dict = {}

        self._input_queue: queue.Queue[str] = queue.Queue()
        self._worker_busy = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # Start each reasoning session with a clean slate of cached photos so
        # scans from a prior run don't linger on disk or leak into context.
        removed = 0
        for p in glob.glob('/tmp/roomba_view.jpg') + glob.glob('/tmp/roomba_scan_*.jpg'):
            try:
                os.remove(p)
                removed += 1
            except OSError:
                pass

        self.get_logger().info(
            f'reasoning_node ready (model={CLAUDE_MODEL}, '
            f'effort={"on" if USE_EFFORT else "off"}, '
            f'cleaned {removed} stale photo(s))'
        )

    # ---------- ROS callbacks (main executor thread) ----------

    def _on_input(self, msg: String) -> None:
        text = msg.data

        # If the operator is trying to stop/cancel and we're mid-action,
        # publish cancel directly so the in-flight tool aborts NOW. The
        # worker's _dispatch_and_wait will wake up with a canceled status
        # and the agentic loop will unwind naturally — no extra threading.
        if self._worker_busy and self._starts_with_stop(text):
            self._publish_command({'action': 'cancel'})
            self._publish_response('🛑 canceling current action')
            # Still queue the message in case it's compound ("stop and go
            # to the kitchen") — Claude will process the rest when the
            # current agentic loop finishes unwinding.
            self._input_queue.put(text)
            return

        # Normal path — let the operator know we got it if we're busy.
        if self._worker_busy or not self._input_queue.empty():
            ahead = self._input_queue.qsize()
            if ahead == 0:
                note = '⏳ got it — I\'ll get to that after the current task.'
            else:
                note = f'⏳ queued ({ahead + 1} ahead of this one).'
            self._publish_response(note)
        self._input_queue.put(text)

    @staticmethod
    def _starts_with_stop(text: str) -> bool:
        """True if the message is (starts with) a stop/cancel directive."""
        cleaned = (text or '').strip().lower().rstrip('.!?,')
        if not cleaned:
            return False
        words = cleaned.split()
        first = words[0] if words else ''
        two = ' '.join(words[:2]) if len(words) >= 2 else ''
        return (
            first in {'stop', 'cancel', 'halt', 'wait', 'pause', 'abort',
                      'nevermind'}
            or two in {'never mind', 'hold on', 'forget it'}
        )

    def _on_bump_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._bump_lock:
            self._last_bump_ns = int(event.get('timestamp_ns', 0)) or \
                self.get_clock().now().nanoseconds
            self._last_bump_info = event

    def _recent_bump(self, window_s: float = 3.0) -> dict | None:
        """Return the most recent bump event if it occurred within window_s."""
        with self._bump_lock:
            if not self._last_bump_info or not self._last_bump_ns:
                return None
            age_s = (self.get_clock().now().nanoseconds - self._last_bump_ns) / 1e9
            if age_s > window_s:
                return None
            return dict(self._last_bump_info)

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        action = payload.get('action')
        state = payload.get('state')
        # success / failed / canceled are all terminal — wake any waiters.
        if not action or state not in ('success', 'failed', 'canceled'):
            return
        with self._status_lock:
            self._status_payloads[action] = payload
            ev = self._status_events.get(action)
            if ev is not None:
                ev.set()

    # ---------- worker (Claude + dispatch) ----------

    def _worker_loop(self) -> None:
        while rclpy.ok():
            try:
                user_input = self._input_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self._worker_busy = True
            try:
                self._process(user_input)
            except Exception as e:
                self.get_logger().error(f'reasoning error: {e}')
                self._publish_response(f'(internal error: {e})')
            finally:
                self._worker_busy = False

    def _process(self, user_input: str) -> None:
        self.get_logger().info(f'user: {user_input}')
        self.messages.append({'role': 'user', 'content': user_input})

        # Agentic loop — call Claude, execute any tools, repeat.
        while True:
            create_kwargs: dict[str, Any] = {
                'model': CLAUDE_MODEL,
                'max_tokens': 4096,
                'system': SYSTEM_PROMPT,
                'tools': TOOLS,
                'thinking': {'type': 'adaptive'},
                'messages': self.messages,
            }
            if USE_EFFORT:
                create_kwargs['output_config'] = {'effort': 'medium'}
            response = self.client.messages.create(**create_kwargs)

            # Record assistant turn verbatim (including thinking + tool_use blocks).
            self.messages.append({'role': 'assistant', 'content': response.content})

            # Emit any user-facing text now.
            text_parts = [b.text for b in response.content if b.type == 'text']
            if text_parts:
                self._publish_response('\n'.join(text_parts))

            if response.stop_reason != 'tool_use':
                break

            # Execute every tool_use block, collect tool_result blocks.
            tool_results = []
            for block in response.content:
                if block.type != 'tool_use':
                    continue
                self.get_logger().info(f'tool: {block.name}({block.input})')
                content = self._execute_tool(block.name, block.input)
                tool_results.append({
                    'type': 'tool_result',
                    'tool_use_id': block.id,
                    'content': content,
                })

            self.messages.append({'role': 'user', 'content': tool_results})

    # ---------- tool execution ----------

    def _execute_tool(self, name: str, tool_input: dict) -> Any:
        if name == 'take_photo':
            share = tool_input.get('share', True)
            return self._tool_take_photo(share=bool(share))
        if name == 'scan_surroundings':
            try:
                n = int(tool_input.get('num_photos', 6))
            except (TypeError, ValueError):
                n = 6
            n = max(2, min(12, n))
            share = tool_input.get('share', True)
            return self._tool_scan_surroundings(n, share=bool(share))
        if name == 'get_state':
            return self._tool_get_state()
        if name == 'list_locations':
            return self._tool_list_locations()
        if name == 'where_am_i':
            return self._tool_where_am_i()
        if name == 'battery_status':
            return self._tool_battery_status()
        if name == 'describe_map':
            return self._tool_describe_map()
        if name == 'face':
            payload: dict[str, Any] = {'action': 'face'}
            if tool_input.get('target'):
                payload['target'] = tool_input['target']
            if tool_input.get('x') is not None:
                payload['x'] = tool_input['x']
            if tool_input.get('y') is not None:
                payload['y'] = tool_input['y']
            if tool_input.get('heading_deg') is not None:
                payload['heading_deg'] = tool_input['heading_deg']
            return self._await_action(payload, 'face')
        if name == 'goto_pose':
            try:
                x = float(tool_input.get('x'))
                y = float(tool_input.get('y'))
            except (TypeError, ValueError):
                return 'error: x and y must be numbers'
            try:
                yaw_deg = float(tool_input.get('yaw_deg', 0.0))
            except (TypeError, ValueError):
                yaw_deg = 0.0
            return self._await_action(
                {
                    'action': 'goto_pose',
                    'x': x, 'y': y,
                    'yaw_rad': math.radians(yaw_deg),
                },
                'goto_pose',
            )
        if name == 'cancel':
            # Cancel is intentionally fire-and-forget — we want to stop the
            # current action, not wait for anything. action_executor will
            # publish a terminal status for whatever was active.
            self._publish_command({'action': 'cancel'})
            return 'cancel dispatched'
        if name in ('goto', 'look_around', 'dock', 'undock'):
            payload: dict[str, Any] = {'action': name}
            if name == 'goto':
                target = tool_input.get('target')
                if not target:
                    return 'error: missing target for goto'
                payload['target'] = target
            return self._await_action(payload, name)
        if name == 'move_forward':
            try:
                dist = float(tool_input.get('distance_m'))
            except (TypeError, ValueError):
                return 'error: distance_m must be a number'
            if dist <= 0:
                return 'error: distance_m must be > 0'
            return self._await_action(
                {'action': 'move_forward', 'distance_m': dist}, 'move_forward'
            )
        if name == 'move_backward':
            try:
                dist = float(tool_input.get('distance_m'))
            except (TypeError, ValueError):
                return 'error: distance_m must be a number'
            if dist <= 0:
                return 'error: distance_m must be > 0 (direction is implied)'
            return self._await_action(
                {'action': 'move_backward', 'distance_m': dist}, 'move_backward'
            )
        if name == 'rotate':
            try:
                angle_deg = float(tool_input.get('angle_deg'))
            except (TypeError, ValueError):
                return 'error: angle_deg must be a number'
            return self._await_action(
                {'action': 'rotate', 'angle_rad': math.radians(angle_deg)},
                'rotate',
            )
        if name == 'save_location':
            loc_name = tool_input.get('name')
            if not loc_name:
                return 'error: missing name for save_location'
            payload: dict[str, Any] = {
                'action': 'save_location',
                'name': loc_name,
            }
            if tool_input.get('facing_target'):
                payload['facing_target'] = tool_input['facing_target']
            if tool_input.get('facing_x') is not None:
                payload['facing_x'] = tool_input['facing_x']
            if tool_input.get('facing_y') is not None:
                payload['facing_y'] = tool_input['facing_y']
            # Explicit-pose inputs (skip TF, register a waypoint without
            # being there).
            if tool_input.get('x') is not None:
                payload['x'] = tool_input['x']
            if tool_input.get('y') is not None:
                payload['y'] = tool_input['y']
            if tool_input.get('yaw_deg') is not None:
                try:
                    payload['yaw_rad'] = math.radians(float(tool_input['yaw_deg']))
                except (TypeError, ValueError):
                    return 'error: yaw_deg must be a number'
            if tool_input.get('type') in ('position', 'viewpoint'):
                payload['type'] = tool_input['type']
            return self._await_action(payload, 'save_location')
        if name == 'delete_location':
            loc_name = tool_input.get('name')
            if not loc_name:
                return 'error: missing name for delete_location'
            return self._await_action(
                {'action': 'delete_location', 'name': loc_name},
                'delete_location',
            )
        return f'error: unknown tool {name!r}'

    def _await_action(self, cmd: dict, action_name: str) -> str:
        """Dispatch an async action and block until it terminates."""
        timeout = ACTION_TIMEOUTS.get(action_name, 30.0)
        payload = self._dispatch_and_wait(cmd, wait_for=action_name, timeout_s=timeout)
        if payload is None:
            return (
                f'error: {action_name} did not complete within {timeout:.0f}s '
                f'(status topic silent) — the action may still be running'
            )
        state = payload.get('state', 'unknown')
        message = payload.get('message', '')
        # Build a concise, model-readable summary of the outcome.
        summary = f'{action_name} {state}'
        if message:
            summary += f': {message}'
        # Include useful extras (e.g. goto target, look_around distance) when present.
        for k in ('target', 'distance_remaining'):
            if k in payload:
                summary += f' [{k}={payload[k]}]'

        # If this was a motion action that got canceled, and a bump
        # interrupt fired within the last few seconds, annotate so Claude
        # can reason about cause and recovery.
        if state == 'canceled' and action_name in (
            'goto', 'goto_pose', 'move_forward', 'move_backward',
            'rotate', 'look_around',
        ):
            bump = self._recent_bump(window_s=3.0)
            if bump is not None:
                hazard = bump.get('hazard', 'unknown')
                pt = bump.get('contact_point_base_link', [0.0, 0.0, 0.0])
                summary += (
                    f' — caused by {hazard} sensor '
                    f'(contact at x={pt[0]:.2f}m, y={pt[1]:.2f}m in base_link, '
                    'i.e. directly in front). The robot has already reversed '
                    '~15cm. Follow the bump-recovery pattern — take a silent '
                    'photo, identify the obstacle\'s side, plan an explicit '
                    'escape (rotate + step sideways), then retry the goto.'
                )
        return summary

    def _tool_list_locations(self) -> str:
        payload = self._dispatch_and_wait({'action': 'list'}, wait_for='list')
        if payload is None:
            return 'error: list command timed out'
        locs = payload.get('locations') or []
        if not locs:
            return 'no named locations are saved yet'
        return 'saved locations: ' + ', '.join(locs)

    def _tool_get_state(self) -> str:
        payload = self._dispatch_and_wait(
            {'action': 'get_state'}, wait_for='get_state',
            timeout_s=ACTION_TIMEOUTS['get_state'],
        )
        if payload is None:
            return 'error: get_state timed out'
        if payload.get('state') != 'success':
            return f"error: {payload.get('message', 'failed')}"
        bits = []
        if 'x' in payload:
            bits.append(
                f"position ({payload['x']}, {payload['y']})m, heading {payload['yaw_deg']}°"
            )
        if 'nearest_location' in payload:
            bits.append(
                f"nearest named location: {payload['nearest_location']} "
                f"({payload['nearest_location_m']}m)"
            )
        lc = payload.get('lidar_clearance')
        if lc:
            parts = []
            for k, label in (('front_m', 'front'), ('left_m', 'left'),
                             ('right_m', 'right'), ('back_m', 'back')):
                v = lc.get(k)
                if v is not None:
                    parts.append(f'{label} {v}m')
                else:
                    parts.append(f'{label} unknown')
            bits.append('lidar clearance: ' + ', '.join(parts))
        if 'roomba_battery_percent' in payload:
            bits.append(f"roomba battery {payload['roomba_battery_percent']}%")
        if 'pi_battery_percent' in payload:
            bits.append(f"pi battery {payload['pi_battery_percent']}%")
        if payload.get('on_dock'):
            bits.append('on dock')
        return '; '.join(bits) if bits else 'no data available'

    def _tool_where_am_i(self) -> str:
        payload = self._dispatch_and_wait(
            {'action': 'where_am_i'}, wait_for='where_am_i',
            timeout_s=ACTION_TIMEOUTS['where_am_i'],
        )
        if payload is None:
            return 'error: where_am_i timed out'
        if payload.get('state') != 'success':
            return f"error: {payload.get('message', 'failed')}"
        bits = [
            f"position ({payload.get('x')}, {payload.get('y')})m",
            f"heading {payload.get('yaw_deg')}°",
        ]
        if 'nearest_location' in payload:
            bits.append(
                f"nearest: {payload['nearest_location']} "
                f"({payload.get('nearest_distance_m')}m)"
            )
        nearby = payload.get('nearby_locations') or []
        others = [e for e in nearby if e['name'] != payload.get('nearest_location')]
        if others:
            bits.append(
                'also within 3m: ' +
                ', '.join(f"{e['name']} ({e['distance_m']}m)" for e in others)
            )
        return '; '.join(bits)

    def _tool_describe_map(self) -> str:
        payload = self._dispatch_and_wait(
            {'action': 'describe_map'}, wait_for='describe_map',
            timeout_s=ACTION_TIMEOUTS['describe_map'],
        )
        if payload is None:
            return 'error: describe_map timed out'
        if payload.get('state') != 'success':
            return f"error: {payload.get('message', 'failed')}"

        lines = []
        b = payload.get('bounds') or {}
        size = payload.get('size_m') or [0, 0]
        lines.append(
            f"Map: {size[0]}×{size[1]}m — "
            f"x from {b.get('min_x')} to {b.get('max_x')}m, "
            f"y from {b.get('min_y')} to {b.get('max_y')}m, "
            f"resolution {payload.get('resolution_m')}m/cell"
        )
        rp = payload.get('robot_pose')
        if rp:
            lines.append(
                f"Robot at ({rp['x']}, {rp['y']}) facing {rp['yaw_deg']}°"
            )
        locs = payload.get('locations') or []
        if locs:
            lines.append('Named locations:')
            for loc in locs:
                lines.append(
                    f"  - {loc['name']}: ({loc['x']}, {loc['y']}), "
                    f"heading {loc['yaw_deg']}°"
                )
        else:
            lines.append('No named locations saved.')
        return '\n'.join(lines)

    def _tool_battery_status(self) -> str:
        payload = self._dispatch_and_wait(
            {'action': 'battery_status'}, wait_for='battery_status',
            timeout_s=ACTION_TIMEOUTS['battery_status'],
        )
        if payload is None:
            return 'error: battery_status timed out'
        if payload.get('state') != 'success':
            return f"error: {payload.get('message', 'failed')}"
        parts = []
        # Roomba pack (drives the motors).
        roomba_bits = [
            f"roomba {payload.get('roomba_charge_percent')}% "
            f"({payload.get('roomba_voltage_v')}V, "
            f"{payload.get('charging_state_desc')}"
        ]
        if payload.get('on_home_base'):
            roomba_bits[0] += ', on dock'
        roomba_bits[0] += ')'
        temp = payload.get('roomba_temperature_c')
        if temp is not None:
            roomba_bits.append(f'temp {temp}°C')
        parts.append('; '.join(roomba_bits))
        # Pi pack (drives the compute).
        pi_v = payload.get('pi_voltage_v')
        pi_pct = payload.get('pi_charge_percent')
        if pi_v is None:
            parts.append('pi battery monitor unavailable')
        else:
            parts.append(f'pi {pi_pct}% ({pi_v}V)')
        return ' | '.join(parts)

    def _tool_scan_surroundings(self, num_photos: int, share: bool = True):
        payload = self._dispatch_and_wait(
            {'action': 'scan_surroundings', 'num_photos': num_photos},
            wait_for='scan_surroundings',
            timeout_s=ACTION_TIMEOUTS['scan_surroundings'],
        )
        if payload is None:
            return 'error: scan_surroundings did not complete in time'
        if payload.get('state') != 'success':
            return f"scan failed: {payload.get('message', 'unknown error')}"

        photos = payload.get('photos') or []
        if not photos:
            return 'scan succeeded but no photos were returned'

        content: list[dict] = []
        start_map_deg = payload.get('start_heading_map_deg')
        if start_map_deg is not None:
            content.append({
                'type': 'text',
                'text': f'Scan started with robot at map heading {start_map_deg}°. '
                        'Each photo below is labelled with its relative angle '
                        '(from scan start) AND absolute map heading. To revisit '
                        'a direction later, call face(heading_deg=<map heading>).',
            })
        for entry in photos:
            angle = entry.get('angle', 0)
            path = entry.get('path')
            abs_map = entry.get('heading_map_deg')
            if not path:
                continue
            # Forward to the operator's Telegram chat only if sharing.
            if share:
                self._publish_photo(path)
            try:
                with open(path, 'rb') as f:
                    img_b64 = base64.b64encode(f.read()).decode('utf-8')
            except OSError as e:
                content.append({'type': 'text', 'text': f'error reading {path}: {e}'})
                continue
            label = f'Photo facing {angle}° rel to scan start (+ = CCW/left)'
            if abs_map is not None:
                label += f', map heading {abs_map}°'
            content.append({'type': 'text', 'text': label})
            content.append({
                'type': 'image',
                'source': {
                    'type': 'base64',
                    'media_type': 'image/jpeg',
                    'data': img_b64,
                },
            })
        return content

    def _tool_take_photo(self, share: bool = True):
        payload = self._dispatch_and_wait(
            {'action': 'take_photo'}, wait_for='take_photo',
            timeout_s=ACTION_TIMEOUTS['take_photo'],
        )
        if payload is None:
            return 'error: take_photo timed out'
        if payload.get('state') == 'failed':
            return f"error: {payload.get('message', 'capture failed')}"
        path = payload.get('path', '/tmp/roomba_view.jpg')
        # Forward the photo to the operator's chat only if the caller asked
        # for it to be shared (default yes, for backward compatibility).
        if share:
            self._publish_photo(path)
        try:
            with open(path, 'rb') as f:
                img_b64 = base64.b64encode(f.read()).decode('utf-8')
        except OSError as e:
            return f'error: could not read image: {e}'
        return [
            {
                'type': 'image',
                'source': {
                    'type': 'base64',
                    'media_type': 'image/jpeg',
                    'data': img_b64,
                },
            }
        ]

    # ---------- publish / wait helpers ----------

    def _publish_command(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.command_pub.publish(msg)

    def _publish_response(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f'assistant: {text}')

    def _publish_photo(self, path: str) -> None:
        msg = String()
        msg.data = path
        self.photo_pub.publish(msg)

    def _dispatch_and_wait(
        self, cmd: dict, *, wait_for: str, timeout_s: float = 30.0
    ) -> dict | None:
        """Publish a command and block until a matching /action/status arrives."""
        event = threading.Event()
        with self._status_lock:
            self._status_events[wait_for] = event
            self._status_payloads.pop(wait_for, None)
        try:
            self._publish_command(cmd)
            if not event.wait(timeout=timeout_s):
                return None
            with self._status_lock:
                return self._status_payloads.get(wait_for)
        finally:
            with self._status_lock:
                self._status_events.pop(wait_for, None)


def main(args=None):
    rclpy.init(args=args)
    node = ReasoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
