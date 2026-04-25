#!/usr/bin/env python3
"""High-level action executor. Exposes named primitives the LLM layer will drive.

Command topic: /action/command  (std_msgs/String, JSON body)
Status topic:  /action/status   (std_msgs/String, JSON body)

Supported commands:
  {"action": "goto",          "target": "<name>"}
  {"action": "save_location", "name":   "<name>"}
  {"action": "look_around"}
  {"action": "dock"}
  {"action": "undock"}
  {"action": "cancel"}
  {"action": "list"}
"""

import json
import math
import os
import subprocess
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


# --- Dock IR bit flags (iRobot OI: packets 17/52/53) ---
# Every genuine dock IR character has high nibble 0xA0. Within the low nibble,
# bit 0 = Force Field, bit 2 = Green Buoy, bit 3 = Red Buoy.
DOCK_HIGH_NIBBLE = 0xA0
DOCK_FORCE_FIELD = 0x01
DOCK_GREEN_BUOY  = 0x04
DOCK_RED_BUOY    = 0x08

# --- IR docking speeds ---
# Turn rates are bumped above the default to get real power at the wheels
# after the 30mm/s deadband floor in roomba_node.
DOCK_FORWARD_SPEED     = 0.08   # m/s in the red+green overlap corridor
DOCK_ARC_FORWARD       = 0.04   # m/s while arcing toward centerline
DOCK_ARC_TURN          = 0.55   # rad/s turn rate during an arc
DOCK_FINAL_SPEED       = 0.04   # m/s during final force-field approach
DOCK_WIGGLE_RATE       = 1.2    # rad/s during force-field wiggle
DOCK_WIGGLE_PERIOD_S   = 0.5    # seconds per wiggle half-cycle
DOCK_SEARCH_SPEED      = 0.5    # rad/s in-place search spin
DOCK_TIMEOUT_S         = 120.0

# Teleop parking window (simplified dock sequence): how long to leave the
# bump sensors disabled while the user drives onto the dock manually.
DOCK_TELEOP_WINDOW_S   = 60.0

# Undock.
UNDOCK_SPEED_S         = 0.15
UNDOCK_DURATION_S      = 4.0   # ~60cm reverse — enough to clear dock footprint

# Fine-grained motion (Nav2 behavior actions).
MOTION_LINEAR_SPEED    = 0.15   # m/s for move_forward / move_backward
MOTION_TIME_ALLOWANCE  = 30.0   # seconds max per motion action

# Pi battery (3S Li-ion via INA219 at I2C 0x41).
PI_BATT_MIN_V          = 9.0    # 3.0V/cell — empty
PI_BATT_MAX_V          = 12.6   # 4.2V/cell — full

# Panoramic scan.
SCAN_DEFAULT_PHOTOS    = 6
SCAN_TURN_SPEED        = 0.8    # rad/s while rotating between captures
SCAN_SETTLE_S          = 0.5    # pause before each capture so frame isn't blurred
SCAN_ANGLE_TOL_RAD     = math.radians(2.0)   # acceptable error per turn


DEFAULT_LOCATIONS_FILE = os.path.expanduser(
    '~/ros2_ws/src/roomba/config/locations.yaml'
)

FILE_HEADER = (
    "# Named locations on the map. Poses are in the map frame.\n"
    "# x, y in meters; yaw in radians (0 = +X, pi/2 = +Y).\n"
    "#\n"
    "# Populate by driving to a spot and calling save_location:\n"
    "#   ros2 topic pub --once /action/command std_msgs/String \\\n"
    "#     \"data: '{\\\"action\\\": \\\"save_location\\\", "
    "\\\"name\\\": \\\"kitchen\\\"}'\"\n"
    "#\n"
    "# This file is rewritten on each save.\n\n"
)


def yaw_from_quaternion(q) -> float:
    """Extract yaw (Z-rotation) from a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        self.declare_parameter('locations_file', DEFAULT_LOCATIONS_FILE)
        self.locations_file = (
            self.get_parameter('locations_file').get_parameter_value().string_value
        )

        self.locations = self._load_locations()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.drive_on_heading_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')

        # Only one primitive runs at a time. If a new one arrives while
        # another is active, we reject it — the caller should cancel first.
        self._active_handle = None
        self._active_action = None

        self.status_pub = self.create_publisher(String, 'action/status', 10)
        self.cmd_sub = self.create_subscription(
            String, 'action/command', self._on_command, 10
        )

        # --- Direct-drive plumbing for dock/undock (bypasses Nav2) ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sensor_sub = self.create_subscription(
            String, '/roomba/sensors', self._on_sensors, 10
        )
        self.bump_enable_client = self.create_client(
            SetBool, '/bump_obstacle/set_enabled'
        )

        self._latest_sensors: dict = {}

        # /map comes from slam_toolbox in localization mode and is latched.
        self._latest_map_info: dict | None = None
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, latched_qos
        )

        # Keep the most recent lidar scan so get_state can produce a quick
        # "what's around me" summary without taking a photo.
        self._latest_scan: LaserScan | None = None
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10
        )

        # Dock/undock state. None when idle.
        self._dock_state: str | None = None            # 'ir_docking' | 'undocking'
        self._dock_deadline_ns: int = 0                # timeout for ir_docking
        self._undock_until_ns: int = 0                 # end-time for undocking
        self._dock_timer = self.create_timer(0.1, self._dock_tick)

        # Panoramic scan is driven by a worker thread so it doesn't block
        # ROS callbacks. True while the scan loop is running.
        self._scan_active = False

        # Pi battery monitor (INA219 on I2C shared with IMU). Optional — if
        # unavailable we still report Roomba battery, just without Pi info.
        self._ina219 = None
        try:
            from batteries.INA219 import INA219
            self._ina219 = INA219(addr=0x41)
            self.get_logger().info('INA219 Pi battery monitor connected')
        except Exception as e:
            self.get_logger().warn(f'INA219 unavailable (Pi battery disabled): {e}')

        self.get_logger().info(
            f'action_executor ready. '
            f'{len(self.locations)} location(s) loaded from {self.locations_file}'
        )

    # ---------- YAML I/O ----------

    def _load_locations(self) -> dict:
        if not os.path.exists(self.locations_file):
            return {}
        with open(self.locations_file) as f:
            data = yaml.safe_load(f) or {}
        locs = data.get('locations') or {}
        cleaned = {}
        for name, pose in locs.items():
            try:
                entry = {
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                    'yaw': float(pose['yaw']),
                }
                # Type controls whether goto orients to the saved yaw at the
                # end. Viewpoint = yes (face the target). Position = no
                # (robot stops in whatever direction it arrived). Explicit
                # field wins; otherwise infer from name suffix for entries
                # saved before this convention existed.
                if 'type' in pose:
                    entry['type'] = (
                        'viewpoint' if pose['type'] == 'viewpoint' else 'position'
                    )
                elif name.endswith('_view') or name.endswith('_spot'):
                    entry['type'] = 'viewpoint'
                else:
                    entry['type'] = 'position'
                cleaned[name] = entry
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().warning(f'Skipping malformed location {name!r}: {e}')
        return cleaned

    def _save_locations(self) -> None:
        os.makedirs(os.path.dirname(self.locations_file), exist_ok=True)
        body = yaml.safe_dump(
            {'locations': self.locations},
            sort_keys=True,
            default_flow_style=False,
        )
        tmp = self.locations_file + '.tmp'
        with open(tmp, 'w') as f:
            f.write(FILE_HEADER)
            f.write(body)
        os.replace(tmp, self.locations_file)

    # ---------- status helper ----------

    def _status(self, action: str, state: str, message: str = '', **extras) -> None:
        payload = {'action': action, 'state': state, 'message': message}
        payload.update(extras)
        self.status_pub.publish(String(data=json.dumps(payload)))
        log = self.get_logger()
        line = f'[{action}] {state}: {message}' if message else f'[{action}] {state}'
        (log.warn if state == 'failed' else log.info)(line)

    # ---------- command dispatch ----------

    def _on_command(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._status('unknown', 'failed', f'invalid JSON: {e}')
            return

        action = cmd.get('action')
        if action == 'goto':
            self._do_goto(cmd.get('target'))
        elif action == 'save_location':
            self._do_save_location(
                cmd.get('name'),
                facing_target=cmd.get('facing_target'),
                facing_x=cmd.get('facing_x'),
                facing_y=cmd.get('facing_y'),
                explicit_x=cmd.get('x'),
                explicit_y=cmd.get('y'),
                explicit_yaw_rad=cmd.get('yaw_rad'),
                explicit_type=cmd.get('type'),
            )
        elif action == 'look_around':
            self._do_look_around()
        elif action == 'dock':
            self._do_dock()
        elif action == 'undock':
            self._do_undock()
        elif action == 'take_photo':
            self._do_take_photo()
        elif action == 'move_forward':
            self._do_move_forward(cmd.get('distance_m'))
        elif action == 'move_backward':
            self._do_move_backward(cmd.get('distance_m'))
        elif action == 'rotate':
            self._do_rotate(cmd.get('angle_rad'))
        elif action == 'scan_surroundings':
            self._do_scan_surroundings(cmd.get('num_photos', SCAN_DEFAULT_PHOTOS))
        elif action == 'where_am_i':
            self._do_where_am_i()
        elif action == 'battery_status':
            self._do_battery_status()
        elif action == 'goto_pose':
            self._do_goto_pose(cmd.get('x'), cmd.get('y'), cmd.get('yaw_rad', 0.0))
        elif action == 'describe_map':
            self._do_describe_map()
        elif action == 'face':
            self._do_face(
                cmd.get('target'), cmd.get('x'), cmd.get('y'),
                cmd.get('heading_deg'),
            )
        elif action == 'delete_location':
            self._do_delete_location(cmd.get('name'))
        elif action == 'get_state':
            self._do_get_state()
        elif action == 'cancel':
            self._do_cancel()
        elif action == 'list':
            self._status(
                'list', 'success',
                f'{len(self.locations)} location(s)',
                locations=sorted(self.locations.keys()),
            )
        else:
            self._status(action or 'unknown', 'failed', f'unknown action: {action!r}')

    # ---------- primitives ----------

    def _reject_if_busy(self, action: str) -> bool:
        if self._active_handle is not None:
            self._status(
                action, 'failed',
                f'busy with {self._active_action}; send cancel first',
            )
            return True
        if self._dock_state is not None:
            self._status(
                action, 'failed',
                f'busy with {self._dock_state}; send cancel first',
            )
            return True
        if self._scan_active:
            self._status(
                action, 'failed',
                'busy with scan_surroundings; wait for it to finish',
            )
            return True
        return False

    def _do_goto(self, name: str) -> None:
        if self._reject_if_busy('goto'):
            return
        if not name:
            self._status('goto', 'failed', 'missing target')
            return
        loc = self.locations.get(name)
        if loc is None:
            self._status(
                'goto', 'failed', f'unknown location: {name!r}',
                known=sorted(self.locations.keys()),
            )
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._status('goto', 'failed', 'navigate_to_pose server unavailable')
            return

        # Auto-undock: if we're currently on the charging dock, reverse off
        # before dispatching the Nav2 goal. Nav2 can't plan around the dock
        # (it isn't in the costmap), so starting a nav from the dock gets
        # the robot stuck.
        on_dock = bool((self._latest_sensors.get('34') or {}).get('home_base', False))
        if on_dock:
            thread = threading.Thread(
                target=self._goto_with_auto_undock,
                args=(name, loc),
                daemon=True,
            )
            thread.start()
            return

        self._dispatch_goto(name, loc)

    def _dispatch_goto(self, name: str, loc: dict) -> None:
        """Send a NavigateToPose goal for a resolved named location.

        For viewpoint waypoints, the saved yaw is used as the goal
        orientation — robot ends up facing the saved direction. For
        position waypoints, we use the BEARING from the current pose to
        the target as the goal yaw, so the robot ends up facing the way
        it came in. That keeps Nav2's RotateToGoal critic from forcing
        an end-of-nav rotation just to satisfy a saved yaw the operator
        doesn't actually care about.
        """
        is_viewpoint = loc.get('type') == 'viewpoint'
        if is_viewpoint:
            goal_yaw = float(loc['yaw'])
            yaw_label = 'saved yaw (viewpoint)'
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                cx = float(tf.transform.translation.x)
                cy = float(tf.transform.translation.y)
                goal_yaw = math.atan2(
                    float(loc['y']) - cy, float(loc['x']) - cx
                )
                yaw_label = 'approach bearing (position)'
            except TransformException:
                # Couldn't get current pose — fall back to saved yaw.
                goal_yaw = float(loc['yaw'])
                yaw_label = 'saved yaw (TF unavailable)'

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc['x'])
        goal.pose.pose.position.y = float(loc['y'])
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self._status(
            'goto', 'started',
            f'navigating to {name} ({yaw_label})',
            target=name, x=float(loc['x']), y=float(loc['y']),
            yaw=goal_yaw, location_type=loc.get('type', 'position'),
        )

        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback
        )
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'goto', name)
        )

    def _goto_with_auto_undock(self, name: str, loc: dict) -> None:
        """Reverse off the dock, then dispatch the Nav2 goto goal."""
        self._status(
            'goto', 'started',
            f'on dock — backing off before navigating to {name}',
            target=name,
        )
        # Bumps off while we reverse — the dock's force field otherwise
        # shows up as a sustained bump signal.
        self._set_bump_enabled(False)
        try:
            twist = Twist()
            twist.linear.x = -UNDOCK_SPEED_S
            end = time.monotonic() + UNDOCK_DURATION_S
            while time.monotonic() < end:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
        finally:
            self._set_bump_enabled(True)
        # Now dispatch the nav goal normally.
        self._dispatch_goto(name, loc)

    def _do_goto_pose(self, x, y, yaw_rad) -> None:
        """Navigate to arbitrary (x, y, yaw) coordinates in the map frame."""
        if self._reject_if_busy('goto_pose'):
            return
        try:
            xf = float(x)
            yf = float(y)
            yawf = float(yaw_rad)
        except (TypeError, ValueError):
            self._status('goto_pose', 'failed', 'x, y, yaw_rad must be numbers')
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._status('goto_pose', 'failed', 'navigate_to_pose server unavailable')
            return

        # Same auto-undock logic as the named goto.
        on_dock = bool((self._latest_sensors.get('34') or {}).get('home_base', False))
        if on_dock:
            thread = threading.Thread(
                target=self._goto_pose_with_auto_undock,
                args=(xf, yf, yawf),
                daemon=True,
            )
            thread.start()
            return
        self._dispatch_goto_pose(xf, yf, yawf)

    def _dispatch_goto_pose(self, x: float, y: float, yaw_rad: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        display = f'({x:.2f}, {y:.2f}) @ {math.degrees(yaw_rad):+.0f}°'
        self._status(
            'goto_pose', 'started', f'navigating to {display}',
            x=round(x, 2), y=round(y, 2),
            yaw_deg=round(math.degrees(yaw_rad), 1),
        )
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback
        )
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'goto_pose', display)
        )

    def _goto_pose_with_auto_undock(self, x: float, y: float, yaw_rad: float) -> None:
        display = f'({x:.2f}, {y:.2f})'
        self._status(
            'goto_pose', 'started',
            f'on dock — backing off before navigating to {display}',
        )
        self._set_bump_enabled(False)
        try:
            twist = Twist()
            twist.linear.x = -UNDOCK_SPEED_S
            end = time.monotonic() + UNDOCK_DURATION_S
            while time.monotonic() < end:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
            self.cmd_vel_pub.publish(Twist())
        finally:
            self._set_bump_enabled(True)
        self._dispatch_goto_pose(x, y, yaw_rad)

    def _do_face(self, target, x, y, heading_deg=None) -> None:
        """Rotate in place to face a named location, raw coords, or an
        absolute map heading."""
        if self._reject_if_busy('face'):
            return

        # Current pose — needed for all three input modes.
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except TransformException as e:
            self._status('face', 'failed', f'no map->base_link TF yet: {e}')
            return
        cx = float(tf.transform.translation.x)
        cy = float(tf.transform.translation.y)
        cyaw = yaw_from_quaternion(tf.transform.rotation)

        # Resolve the desired bearing (absolute map yaw to rotate to).
        if target:
            loc = self.locations.get(target)
            if loc is None:
                self._status(
                    'face', 'failed', f'unknown location: {target!r}',
                    known=sorted(self.locations.keys()),
                )
                return
            tx, ty = float(loc['x']), float(loc['y'])
            bearing = math.atan2(ty - cy, tx - cx)
            label = str(target)
        elif x is not None and y is not None:
            try:
                tx = float(x)
                ty = float(y)
            except (TypeError, ValueError):
                self._status('face', 'failed', 'x, y must be numbers')
                return
            bearing = math.atan2(ty - cy, tx - cx)
            label = f'({tx:.2f}, {ty:.2f})'
        elif heading_deg is not None:
            try:
                bearing = math.radians(float(heading_deg))
            except (TypeError, ValueError):
                self._status('face', 'failed', 'heading_deg must be a number')
                return
            label = f'map heading {heading_deg}°'
        else:
            self._status(
                'face', 'failed',
                'specify target (name), x+y (coords), or heading_deg (map yaw)',
            )
            return

        # Signed shortest rotation from current yaw to the desired bearing.
        delta = bearing - cyaw
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi

        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self._status('face', 'failed', 'spin server unavailable')
            return
        goal = Spin.Goal()
        goal.target_yaw = delta
        goal.time_allowance = Duration(seconds=MOTION_TIME_ALLOWANCE).to_msg()
        self._status(
            'face', 'started',
            f'rotating {math.degrees(delta):+.0f}° to face {label}',
            target=label,
            angle_deg=round(math.degrees(delta), 1),
            bearing_deg=round(math.degrees(bearing), 1),
        )
        send_future = self.spin_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'face', label)
        )

    def _do_delete_location(self, name) -> None:
        if not name:
            self._status('delete_location', 'failed', 'missing name')
            return
        if name not in self.locations:
            self._status(
                'delete_location', 'failed',
                f'unknown location: {name!r}',
                known=sorted(self.locations.keys()),
            )
            return
        del self.locations[name]
        try:
            self._save_locations()
        except OSError as e:
            self._status('delete_location', 'failed', f'write failed: {e}')
            return
        self._status(
            'delete_location', 'success',
            f'removed {name} ({len(self.locations)} remaining)',
            name=name,
            remaining=sorted(self.locations.keys()),
        )

    def _do_get_state(self) -> None:
        """Compact fast snapshot: pose, nearest location, lidar clearances,
        batteries, dock status. Intended as the first thing Claude calls
        before planning any action."""
        state: dict = {}

        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            yaw = yaw_from_quaternion(tf.transform.rotation)
            state['x'] = round(tx, 2)
            state['y'] = round(ty, 2)
            state['yaw_deg'] = round(math.degrees(yaw), 1)

            nearest_name = None
            nearest_dist = float('inf')
            for nm, loc in self.locations.items():
                d = math.hypot(float(loc['x']) - tx, float(loc['y']) - ty)
                if d < nearest_dist:
                    nearest_dist = d
                    nearest_name = nm
            if nearest_name is not None:
                state['nearest_location'] = nearest_name
                state['nearest_location_m'] = round(nearest_dist, 2)
        except TransformException:
            pass

        lidar = self._summarize_lidar()
        if lidar is not None:
            state['lidar_clearance'] = lidar

        # Batteries + dock.
        s = self._latest_sensors
        if s:
            charge_mah = int((s.get('25') or {}).get('battery_charge_mah', 0))
            capacity_mah = int((s.get('26') or {}).get('battery_capacity_mah', 0))
            if capacity_mah:
                state['roomba_battery_percent'] = round(
                    100.0 * charge_mah / capacity_mah, 0
                )
            state['on_dock'] = bool((s.get('34') or {}).get('home_base', False))
        if self._ina219 is not None:
            try:
                pi_v = self._ina219.getBusVoltage_V()
                span = PI_BATT_MAX_V - PI_BATT_MIN_V
                raw_pct = 100.0 * (pi_v - PI_BATT_MIN_V) / span
                state['pi_battery_percent'] = max(0, min(100, round(raw_pct, 0)))
            except Exception:
                pass

        # One-line summary for humans skimming /action/status.
        bits = []
        if 'x' in state:
            bits.append(
                f"at ({state['x']}, {state['y']}) @ {state['yaw_deg']}°"
            )
            if 'nearest_location' in state:
                bits.append(
                    f"near {state['nearest_location']} "
                    f"({state['nearest_location_m']}m)"
                )
        if 'lidar_clearance' in state:
            lc = state['lidar_clearance']
            bits.append(
                f"clearance F{lc['front_m']}/L{lc['left_m']}/"
                f"B{lc['back_m']}/R{lc['right_m']}m"
            )
        if state.get('on_dock'):
            bits.append('on dock')
        if 'roomba_battery_percent' in state:
            bits.append(f"roomba {state['roomba_battery_percent']}%")
        if 'pi_battery_percent' in state:
            bits.append(f"pi {state['pi_battery_percent']}%")
        self._status('get_state', 'success', '; '.join(bits) or 'no data', **state)

    def _do_describe_map(self) -> None:
        """Summarize the map: bounds, resolution, robot pose, named locations."""
        info = self._latest_map_info
        if info is None:
            self._status('describe_map', 'failed', '/map not received yet')
            return

        res = info['resolution_m']
        width_m = info['width_cells'] * res
        height_m = info['height_cells'] * res
        min_x = info['origin_x_m']
        max_x = min_x + width_m
        min_y = info['origin_y_m']
        max_y = min_y + height_m

        robot_pose = None
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            robot_pose = {
                'x': round(float(tf.transform.translation.x), 2),
                'y': round(float(tf.transform.translation.y), 2),
                'yaw_deg': round(
                    math.degrees(yaw_from_quaternion(tf.transform.rotation)), 1
                ),
            }
        except TransformException:
            pass

        locations_list = [
            {
                'name': name,
                'x': round(loc['x'], 2),
                'y': round(loc['y'], 2),
                'yaw_deg': round(math.degrees(loc['yaw']), 0),
            }
            for name, loc in sorted(self.locations.items())
        ]

        summary = (
            f"map {width_m:.1f}×{height_m:.1f}m "
            f"(x [{min_x:.2f},{max_x:.2f}], y [{min_y:.2f},{max_y:.2f}]) "
            f"{res:.2f}m/cell, {len(locations_list)} named location(s)"
        )
        self._status(
            'describe_map', 'success', summary,
            bounds={
                'min_x': round(min_x, 2),
                'max_x': round(max_x, 2),
                'min_y': round(min_y, 2),
                'max_y': round(max_y, 2),
            },
            resolution_m=res,
            size_m=[round(width_m, 2), round(height_m, 2)],
            robot_pose=robot_pose,
            locations=locations_list,
        )

    def _do_look_around(self) -> None:
        if self._reject_if_busy('look_around'):
            return
        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self._status('look_around', 'failed', 'spin server unavailable')
            return
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance = Duration(seconds=30).to_msg()
        self._status('look_around', 'started', 'spinning 360°')
        send_future = self.spin_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'look_around', None)
        )

    def _do_move_forward(self, distance_m) -> None:
        if self._reject_if_busy('move_forward'):
            return
        try:
            distance = float(distance_m)
        except (TypeError, ValueError):
            self._status('move_forward', 'failed', 'distance_m must be a number')
            return
        if distance <= 0:
            self._status('move_forward', 'failed', 'distance_m must be > 0')
            return
        if not self.drive_on_heading_client.wait_for_server(timeout_sec=2.0):
            self._status('move_forward', 'failed', 'drive_on_heading server unavailable')
            return
        goal = DriveOnHeading.Goal()
        goal.target.x = distance
        goal.speed = MOTION_LINEAR_SPEED
        goal.time_allowance = Duration(seconds=MOTION_TIME_ALLOWANCE).to_msg()
        self._status('move_forward', 'started', f'driving forward {distance:.2f}m')
        send_future = self.drive_on_heading_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'move_forward', f'{distance:.2f}m')
        )

    def _do_move_backward(self, distance_m) -> None:
        if self._reject_if_busy('move_backward'):
            return
        try:
            distance = float(distance_m)
        except (TypeError, ValueError):
            self._status('move_backward', 'failed', 'distance_m must be a number')
            return
        if distance <= 0:
            self._status('move_backward', 'failed', 'distance_m must be > 0')
            return
        if not self.backup_client.wait_for_server(timeout_sec=2.0):
            self._status('move_backward', 'failed', 'backup server unavailable')
            return
        goal = BackUp.Goal()
        goal.target.x = distance
        goal.speed = MOTION_LINEAR_SPEED
        goal.time_allowance = Duration(seconds=MOTION_TIME_ALLOWANCE).to_msg()
        self._status('move_backward', 'started', f'reversing {distance:.2f}m')
        send_future = self.backup_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'move_backward', f'{distance:.2f}m')
        )

    def _do_rotate(self, angle_rad) -> None:
        if self._reject_if_busy('rotate'):
            return
        try:
            angle = float(angle_rad)
        except (TypeError, ValueError):
            self._status('rotate', 'failed', 'angle_rad must be a number')
            return
        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self._status('rotate', 'failed', 'spin server unavailable')
            return
        goal = Spin.Goal()
        goal.target_yaw = angle  # positive = CCW/left, negative = CW/right
        goal.time_allowance = Duration(seconds=MOTION_TIME_ALLOWANCE).to_msg()
        deg = math.degrees(angle)
        self._status('rotate', 'started', f'rotating {deg:+.0f}°')
        send_future = self.spin_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, 'rotate', f'{deg:+.0f}°')
        )

    def _do_scan_surroundings(self, num_photos) -> None:
        if self._reject_if_busy('scan_surroundings'):
            return
        try:
            n = int(num_photos)
        except (TypeError, ValueError):
            n = SCAN_DEFAULT_PHOTOS
        n = max(2, min(12, n))
        self._scan_active = True
        self._set_bump_enabled(False)
        self._status(
            'scan_surroundings', 'started',
            f'spinning and capturing {n} photos',
        )
        thread = threading.Thread(
            target=self._scan_worker, args=(n,), daemon=True
        )
        thread.start()

    def _scan_worker(self, num_photos: int) -> None:
        paths = []
        turn_angle = 2.0 * math.pi / num_photos
        degrees_per_step = 360.0 / num_photos
        # Capture the starting map-frame yaw so we can label every photo with
        # both a relative angle (convenient) and an absolute map heading
        # (stable — survives the robot driving elsewhere later).
        try:
            tf0 = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            start_yaw_map_rad = yaw_from_quaternion(tf0.transform.rotation)
        except TransformException:
            start_yaw_map_rad = None
        try:
            for i in range(num_photos):
                # Pause for the robot to stop swaying before we capture.
                time.sleep(SCAN_SETTLE_S)

                path = f'/tmp/roomba_scan_{i}.jpg'
                try:
                    subprocess.run(
                        ['ffmpeg', '-y', '-loglevel', 'error',
                         '-f', 'v4l2', '-video_size', '640x480',
                         '-i', '/dev/video0', '-vframes', '1', path],
                        check=True, timeout=10,
                    )
                    entry = {'angle': int(i * degrees_per_step), 'path': path}
                    if start_yaw_map_rad is not None:
                        abs_deg = (math.degrees(start_yaw_map_rad)
                                   + i * degrees_per_step) % 360.0
                        entry['heading_map_deg'] = round(abs_deg, 1)
                    paths.append(entry)
                except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                        FileNotFoundError) as e:
                    self._status('scan_surroundings', 'failed', f'capture error: {e}')
                    return

                # Rotate for the next angle. Run this N times (not N-1) so the
                # robot completes a full 360° and ends facing the original
                # heading — predictable for subsequent commands.
                self._rotate_by_odom(turn_angle, SCAN_TURN_SPEED)

            extras = {'photos': paths}
            if start_yaw_map_rad is not None:
                extras['start_heading_map_deg'] = round(
                    math.degrees(start_yaw_map_rad), 1
                )
            self._status(
                'scan_surroundings', 'success',
                f'captured {len(paths)} photos, returned to original heading',
                **extras,
            )
        finally:
            self.cmd_vel_pub.publish(Twist())
            self._set_bump_enabled(True)
            self._scan_active = False

    def _rotate_by_odom(self, angle_rad: float, speed: float) -> None:
        """Closed-loop in-place rotation using the odom->base_link TF.

        Integrates signed yaw deltas so wrap-around at ±π is handled
        naturally. Stops when the target angle is reached (within
        SCAN_ANGLE_TOL_RAD) or after a safety timeout at 3× the expected
        duration.
        """
        target_mag = abs(angle_rad)
        if target_mag < SCAN_ANGLE_TOL_RAD:
            return
        sign = 1.0 if angle_rad > 0 else -1.0

        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f'scan rotate: TF unavailable ({e}); skipping this turn'
            )
            return
        prev_yaw = yaw_from_quaternion(tf.transform.rotation)
        accumulated = 0.0

        twist = Twist()
        twist.angular.z = sign * speed
        deadline = time.monotonic() + (target_mag / speed) * 3.0 + 1.0

        while abs(accumulated) < target_mag - SCAN_ANGLE_TOL_RAD:
            if time.monotonic() > deadline:
                self.get_logger().warn(
                    f'scan rotate: safety timeout at {math.degrees(accumulated):+.0f}° '
                    f'of target {math.degrees(angle_rad):+.0f}°'
                )
                break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)  # 50Hz

            try:
                tf = self.tf_buffer.lookup_transform(
                    'odom', 'base_link', rclpy.time.Time()
                )
                current_yaw = yaw_from_quaternion(tf.transform.rotation)
                delta = current_yaw - prev_yaw
                # Unwrap: shortest signed path on the ±π circle.
                if delta > math.pi:
                    delta -= 2.0 * math.pi
                elif delta < -math.pi:
                    delta += 2.0 * math.pi
                accumulated += delta
                prev_yaw = current_yaw
            except TransformException:
                # Keep driving; try again next tick.
                pass

        self.cmd_vel_pub.publish(Twist())

    def _do_where_am_i(self) -> None:
        """Report current map-frame pose + nearest named location."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except TransformException as e:
            self._status(
                'where_am_i', 'failed',
                f'no map->base_link TF yet: {e}',
            )
            return
        t = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)

        nearest_name = None
        nearest_dist = float('inf')
        nearby: list[dict] = []
        for name, loc in self.locations.items():
            dx = loc['x'] - float(t.x)
            dy = loc['y'] - float(t.y)
            dist = math.hypot(dx, dy)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_name = name
            if dist < 3.0:
                nearby.append({'name': name, 'distance_m': round(dist, 2)})
        nearby.sort(key=lambda e: e['distance_m'])

        x = round(float(t.x), 2)
        y = round(float(t.y), 2)
        yaw_deg = round(math.degrees(yaw), 1)

        extras: dict = {
            'x': x,
            'y': y,
            'yaw_deg': yaw_deg,
            'nearby_locations': nearby,
        }
        if nearest_name is not None:
            extras['nearest_location'] = nearest_name
            extras['nearest_distance_m'] = round(nearest_dist, 2)

        summary = f'at ({x:.2f}, {y:.2f}) facing {yaw_deg:+.0f}°'
        if nearest_name is not None:
            summary += f'; nearest: {nearest_name} ({nearest_dist:.2f}m)'
        self._status('where_am_i', 'success', summary, **extras)

    def _do_battery_status(self) -> None:
        """Report Roomba + Pi battery state."""
        s = self._latest_sensors
        if not s:
            self._status('battery_status', 'failed', 'no sensor data yet')
            return

        # --- Roomba battery ---
        voltage_mv = int((s.get('22') or {}).get('voltage_mv', 0))
        current_ma = int((s.get('23') or {}).get('current_ma', 0))
        temp_c = int((s.get('24') or {}).get('temperature_c', 0))
        charge_mah = int((s.get('25') or {}).get('battery_charge_mah', 0))
        capacity_mah = int((s.get('26') or {}).get('battery_capacity_mah', 0))
        charging_state = int((s.get('21') or {}).get('charging_state', 0))
        on_home = bool((s.get('34') or {}).get('home_base', False))

        roomba_pct = 100.0 * charge_mah / capacity_mah if capacity_mah else 0.0
        state_desc = {
            0: 'not charging',
            1: 'reconditioning',
            2: 'charging',
            3: 'trickle charging',
            4: 'waiting',
            5: 'charging fault',
        }.get(charging_state, f'unknown ({charging_state})')
        roomba_voltage_v = round(voltage_mv / 1000.0, 2)

        # --- Pi battery (INA219, optional) ---
        pi_voltage_v = None
        pi_percent = None
        if self._ina219 is not None:
            try:
                pi_voltage_v = round(self._ina219.getBusVoltage_V(), 2)
                span = PI_BATT_MAX_V - PI_BATT_MIN_V
                raw_pct = 100.0 * (pi_voltage_v - PI_BATT_MIN_V) / span
                pi_percent = max(0, min(100, round(raw_pct, 0)))
            except Exception as e:
                self.get_logger().warn(f'Pi battery read failed: {e}')

        # --- Compose summary + payload ---
        summary = (
            f'roomba {roomba_pct:.0f}% ({roomba_voltage_v}V, {state_desc}'
            f'{", on dock" if on_home else ""})'
        )
        if pi_voltage_v is not None:
            summary += f' | pi {pi_percent:.0f}% ({pi_voltage_v}V)'

        extras = {
            'roomba_charge_percent': round(roomba_pct, 0),
            'roomba_voltage_v': roomba_voltage_v,
            'roomba_current_ma': current_ma,
            'roomba_temperature_c': temp_c,
            'charging_state': charging_state,
            'charging_state_desc': state_desc,
            'on_home_base': on_home,
        }
        if pi_voltage_v is not None:
            extras['pi_voltage_v'] = pi_voltage_v
            extras['pi_charge_percent'] = pi_percent
        else:
            extras['pi_voltage_v'] = None
            extras['pi_charge_percent'] = None

        self._status('battery_status', 'success', summary, **extras)

    def _do_save_location(self, name: str, facing_target=None,
                          facing_x=None, facing_y=None,
                          explicit_x=None, explicit_y=None,
                          explicit_yaw_rad=None,
                          explicit_type=None) -> None:
        """Save a named waypoint.

        Three input modes:
          (A) Current-pose: pass only `name` (and optional `facing_*`).
              Position is read from TF, yaw is current heading or computed
              to face the target.
          (B) Explicit-pose: pass `name` + `explicit_x` + `explicit_y`
              (+ optional `explicit_yaw_rad`, `explicit_type`). Lets the
              caller register/edit a waypoint without the robot being
              physically there. Useful for editing existing entries or
              entering coords picked off a map.
          The two modes can be mixed with `facing_target` — if explicit
          x/y are given AND facing_target is set, yaw is computed from
          the explicit position to the target.
        """
        if not name:
            self._status('save_location', 'failed', 'missing name')
            return

        # Determine the saved (x, y) — explicit args win over current TF.
        explicit_pose_given = (
            explicit_x is not None and explicit_y is not None
        )
        if explicit_pose_given:
            try:
                cx = float(explicit_x)
                cy = float(explicit_y)
            except (TypeError, ValueError):
                self._status(
                    'save_location', 'failed', 'x, y must be numbers',
                )
                return
            current_yaw_rad = None  # only meaningful if from TF
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
            except TransformException as e:
                self._status(
                    'save_location', 'failed',
                    f'no map->base_link TF yet: {e}',
                )
                return
            cx = float(tf.transform.translation.x)
            cy = float(tf.transform.translation.y)
            current_yaw_rad = yaw_from_quaternion(tf.transform.rotation)

        # Determine the saved yaw + inferred type.
        inferred_type = 'position'
        if facing_target:
            target = self.locations.get(facing_target)
            if target is None:
                self._status(
                    'save_location', 'failed',
                    f'unknown facing_target: {facing_target!r}',
                    known=sorted(self.locations.keys()),
                )
                return
            yaw = math.atan2(
                float(target['y']) - cy, float(target['x']) - cx
            )
            yaw_mode = f'facing {facing_target}'
            inferred_type = 'viewpoint'
        elif facing_x is not None and facing_y is not None:
            try:
                tx = float(facing_x)
                ty = float(facing_y)
            except (TypeError, ValueError):
                self._status(
                    'save_location', 'failed',
                    'facing_x, facing_y must be numbers',
                )
                return
            yaw = math.atan2(ty - cy, tx - cx)
            yaw_mode = f'facing ({tx:.2f}, {ty:.2f})'
            inferred_type = 'viewpoint'
        elif explicit_yaw_rad is not None:
            try:
                yaw = float(explicit_yaw_rad)
            except (TypeError, ValueError):
                self._status(
                    'save_location', 'failed', 'yaw_rad must be a number',
                )
                return
            yaw_mode = 'explicit yaw'
        elif current_yaw_rad is not None:
            yaw = current_yaw_rad
            yaw_mode = 'current heading'
        else:
            # Explicit pose given without yaw and without facing_* — default to 0.
            yaw = 0.0
            yaw_mode = 'no yaw given (default 0)'

        # Type: explicit override wins, otherwise use inferred.
        if explicit_type in ('position', 'viewpoint'):
            loc_type = explicit_type
        else:
            loc_type = inferred_type

        self.locations[name] = {
            'x': cx, 'y': cy, 'yaw': float(yaw), 'type': loc_type,
        }
        try:
            self._save_locations()
        except OSError as e:
            self._status('save_location', 'failed', f'write failed: {e}')
            return
        self._status(
            'save_location', 'success',
            f'saved {name} as {loc_type} at ({cx:.2f}, {cy:.2f}) '
            f'yaw {math.degrees(yaw):+.0f}° ({yaw_mode})',
            name=name, x=cx, y=cy, yaw=float(yaw), type=loc_type,
        )

    def _do_cancel(self) -> None:
        if self._dock_state is not None:
            canceled = self._dock_state
            self._finish_dock_state(
                action=canceled,
                state='canceled',
                message=f'canceled {canceled}',
            )
            return
        if self._active_handle is None:
            self._status('cancel', 'failed', 'no active goal')
            return
        active = self._active_action
        self._active_handle.cancel_goal_async()
        self._status('cancel', 'started', f'canceling {active}')

    # ---------- action plumbing ----------

    def _on_nav_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self._status(
            'goto', 'feedback',
            f'dist_remaining={fb.distance_remaining:.2f}m',
            distance_remaining=float(fb.distance_remaining),
        )

    def _on_goal_response(self, future, action: str, context) -> None:
        handle = future.result()
        if not handle.accepted:
            self._status(action, 'failed', 'goal rejected', target=context)
            return
        self._active_handle = handle
        self._active_action = action
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_goal_result(f, handle, action, context)
        )

    def _on_goal_result(self, future, handle, action: str, context) -> None:
        wrapped = future.result()
        status = wrapped.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._status(
                action, 'success',
                f'reached {context}' if context else 'done',
                target=context,
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self._status(action, 'canceled', 'goal canceled', target=context)
        elif status == GoalStatus.STATUS_ABORTED:
            self._status(action, 'failed', 'goal aborted', target=context)
        else:
            self._status(action, 'failed', f'unexpected status={status}', target=context)
        if self._active_handle is handle:
            self._active_handle = None
            self._active_action = None

    # ---------- dock / undock ----------

    def _on_sensors(self, msg: String) -> None:
        try:
            self._latest_sensors = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map_info = {
            'width_cells': msg.info.width,
            'height_cells': msg.info.height,
            'resolution_m': float(msg.info.resolution),
            'origin_x_m': float(msg.info.origin.position.x),
            'origin_y_m': float(msg.info.origin.position.y),
        }

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _summarize_lidar(self) -> dict | None:
        """Nearest-obstacle distance in each quadrant around the robot.

        The lidar is mounted rotated 180° from base_link (yaw=π in the
        static TF), so a point at laser angle θ corresponds to base_link
        angle θ+π. We bucket into front / left / back / right and take the
        min range in each.
        """
        scan = self._latest_scan
        if scan is None or not scan.ranges:
            return None
        narrow = math.radians(20.0)  # ± this defines front/back
        front: list[float] = []
        left: list[float] = []
        back: list[float] = []
        right: list[float] = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            laser_angle = scan.angle_min + i * scan.angle_increment
            # Convert to base_link frame (yaw offset = +π).
            a = laser_angle + math.pi
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            if abs(a) < narrow:
                front.append(r)
            elif abs(a) > math.pi - narrow:
                back.append(r)
            elif a > 0:
                left.append(r)
            else:
                right.append(r)

        def nearest(rs: list[float]) -> float | None:
            return round(min(rs), 2) if rs else None

        return {
            'front_m': nearest(front),
            'left_m': nearest(left),
            'back_m': nearest(back),
            'right_m': nearest(right),
        }

    def _do_dock(self) -> None:
        if self._reject_if_busy('dock'):
            return
        # Simplified: disable bump protection and give the user a fixed window
        # to teleop onto the dock manually. Bumps re-enable automatically.
        self._set_bump_enabled(False)
        self._dock_state = 'teleop_park'
        self._dock_deadline_ns = (
            self.get_clock().now().nanoseconds + int(DOCK_TELEOP_WINDOW_S * 1e9)
        )
        self._status(
            'dock', 'started',
            f'bumps disabled for {DOCK_TELEOP_WINDOW_S:.0f}s — drive onto the dock manually',
        )

    def _do_take_photo(self) -> None:
        """Grab a single frame from /dev/video0 via ffmpeg."""
        out_path = '/tmp/roomba_view.jpg'
        try:
            subprocess.run(
                ['ffmpeg', '-y', '-loglevel', 'error',
                 '-f', 'v4l2', '-video_size', '640x480',
                 '-i', '/dev/video0', '-vframes', '1', out_path],
                check=True, timeout=10,
            )
        except subprocess.CalledProcessError as e:
            self._status('take_photo', 'failed', f'ffmpeg failed: {e}')
            return
        except subprocess.TimeoutExpired:
            self._status('take_photo', 'failed', 'ffmpeg timed out')
            return
        except FileNotFoundError:
            self._status('take_photo', 'failed', 'ffmpeg not installed')
            return
        self._status(
            'take_photo', 'success', f'captured image to {out_path}',
            path=out_path,
        )

    def _do_undock(self) -> None:
        if self._reject_if_busy('undock'):
            return
        self._set_bump_enabled(False)
        self._dock_state = 'undocking'
        self._undock_until_ns = (
            self.get_clock().now().nanoseconds + int(UNDOCK_DURATION_S * 1e9)
        )
        self._status(
            'undock', 'started',
            f'reversing {UNDOCK_DURATION_S:.1f}s at {UNDOCK_SPEED_S:.2f} m/s',
        )

    def _set_bump_enabled(self, enabled: bool) -> None:
        if not self.bump_enable_client.service_is_ready():
            self.get_logger().warn(
                'bump_obstacle/set_enabled service not ready — '
                'proceeding but bump protection may still fire'
            )
            return
        req = SetBool.Request()
        req.data = enabled
        self.bump_enable_client.call_async(req)

    def _dock_tick(self) -> None:
        if self._dock_state == 'teleop_park':
            self._teleop_park_tick()
        elif self._dock_state == 'undocking':
            self._undock_tick()

    def _teleop_park_tick(self) -> None:
        if self.get_clock().now().nanoseconds >= self._dock_deadline_ns:
            self._finish_dock_state(
                action='dock',
                state='success',
                message='teleop window ended, bumps re-enabled',
            )

    def _ir_docking_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        # Definitive success: Charging Sources Available reports home_base.
        # This bit is only set when the robot is physically on the dock.
        # charging_state alone is unreliable on this modified hardware —
        # it reports 4 ("Waiting") off-dock as well.
        sources_packet = self._latest_sensors.get('34') or {}
        on_home_base = bool(sources_packet.get('home_base', False))
        charging_packet = self._latest_sensors.get('21') or {}
        charging_state = int(charging_packet.get('charging_state', 0))

        if on_home_base:
            self._publish_stop()
            self._finish_dock_state(
                action='dock',
                state='success',
                message=f'on home base (charging_state={charging_state})',
                charging_state=charging_state,
            )
            return

        # Safety: bumper pressed while seeing force-field means we've hit the
        # dock. Either contacts mated (home_base fires within a few ticks) or
        # we crashed into its back. Stop advancing either way. Give the
        # firmware ~500ms to report home_base (contacts can debounce), then
        # give up if still no charging.
        bump_packet = self._latest_sensors.get('7') or {}
        bumped = bool(bump_packet.get('bump_left') or bump_packet.get('bump_right'))
        _ff_omni  = int((self._latest_sensors.get('17') or {}).get('ir_omni',  0))
        _ff_left  = int((self._latest_sensors.get('52') or {}).get('ir_left',  0))
        _ff_right = int((self._latest_sensors.get('53') or {}).get('ir_right', 0))
        ff_anywhere = any(
            (b & 0xF0) == DOCK_HIGH_NIBBLE and (b & DOCK_FORCE_FIELD)
            for b in (_ff_omni, _ff_left, _ff_right)
        )
        if bumped and ff_anywhere:
            self._publish_stop()
            if self._bump_settle_start_ns == 0:
                self._bump_settle_start_ns = now_ns
            if now_ns - self._bump_settle_start_ns > int(0.5 * 1e9):
                self._finish_dock_state(
                    action='dock',
                    state='failed',
                    message='bumper hit dock but home_base never fired — '
                            'robot likely misaligned with charging contacts',
                )
            return
        else:
            self._bump_settle_start_ns = 0

        # Timeout.
        if now_ns > self._dock_deadline_ns:
            self._publish_stop()
            self._finish_dock_state(
                action='dock',
                state='failed',
                message=f'timeout after {DOCK_TIMEOUT_S:.0f}s',
            )
            return

        # Read all three IR receivers. The omni (17) is unreliable at close
        # range due to dome geometry; the forward-facing left (52) and right
        # (53) are what iRobot uses for final alignment. Union detection.
        omni  = int((self._latest_sensors.get('17') or {}).get('ir_omni',  0))
        left  = int((self._latest_sensors.get('52') or {}).get('ir_left',  0))
        right = int((self._latest_sensors.get('53') or {}).get('ir_right', 0))

        # Only trust bytes that look like genuine dock signals (high nibble A).
        # Rejects TV remotes, virtual walls (0xA2), lighthouses, etc.
        def dock_bits(b: int) -> int:
            return b & (DOCK_RED_BUOY | DOCK_GREEN_BUOY | DOCK_FORCE_FIELD) \
                if (b & 0xF0) == DOCK_HIGH_NIBBLE else 0

        bits = dock_bits(omni) | dock_bits(left) | dock_bits(right)
        red         = bool(bits & DOCK_RED_BUOY)
        green       = bool(bits & DOCK_GREEN_BUOY)
        force_field = bool(bits & DOCK_FORCE_FIELD)

        twist = Twist()
        phase: str

        if force_field and not red and not green:
            # Inside ~0.5m force-field bubble with no clean beam lock.
            # Native behavior: wiggle laterally (ILS-style) while creeping
            # forward to sweep across the dock centerline.
            t = now_ns / 1e9
            sign = 1.0 if int(t / DOCK_WIGGLE_PERIOD_S) % 2 == 0 else -1.0
            twist.linear.x  = DOCK_FINAL_SPEED
            twist.angular.z = sign * DOCK_WIGGLE_RATE
            phase = 'wiggle'
        elif red and green:
            # Aligned with dock — drive straight. Slower if also in force-field.
            twist.linear.x = DOCK_FINAL_SPEED if force_field else DOCK_FORWARD_SPEED
            phase = 'overlap_ff' if force_field else 'overlap'
        elif red:
            # Red-only → dock is to our left. Arc forward while turning left.
            twist.linear.x  = DOCK_ARC_FORWARD
            twist.angular.z = DOCK_ARC_TURN
            phase = 'arc_left'
        elif green:
            # Green-only → dock is to our right. Arc forward while turning right.
            twist.linear.x  = DOCK_ARC_FORWARD
            twist.angular.z = -DOCK_ARC_TURN
            phase = 'arc_right'
        else:
            # No beams at all. Spin slowly, preserving last turn direction
            # so brief dropouts don't fling us the opposite way.
            twist.angular.z = self._dock_last_turn_sign * DOCK_SEARCH_SPEED
            phase = 'search'

        # Remember the last non-zero turn direction for search-spin continuity.
        if twist.angular.z > 0.0:
            self._dock_last_turn_sign = 1.0
        elif twist.angular.z < 0.0:
            self._dock_last_turn_sign = -1.0

        self.cmd_vel_pub.publish(twist)

        # Throttled telemetry so you can see what each sensor is reporting.
        if now_ns - self._last_dock_feedback_ns > int(0.5 * 1e9):
            self._last_dock_feedback_ns = now_ns
            self._status(
                'dock', 'feedback',
                f'{phase} o=0x{omni:02x} l=0x{left:02x} r=0x{right:02x}',
                phase=phase, omni=omni, ir_left=left, ir_right=right,
            )

    def _undock_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._undock_until_ns:
            twist = Twist()
            twist.linear.x = -UNDOCK_SPEED_S
            self.cmd_vel_pub.publish(twist)
            return
        # Done.
        self._publish_stop()
        self._finish_dock_state(
            action='undock',
            state='success',
            message='undocked',
        )

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _finish_dock_state(self, *, action: str, state: str, message: str, **extras) -> None:
        """End a dock/undock run and restore normal operation."""
        self._publish_stop()
        self._dock_state = None
        self._set_bump_enabled(True)
        self._status(action, state, message, **extras)


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()
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
