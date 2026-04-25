#!/usr/bin/env python3
"""Bump + cliff -> emergency stop, reverse, mark obstacle.

On any bump or cliff rising edge (with a cooldown):
  1. Cancel all active Nav2 goals.
  2. Drive reverse for REVERSE_DURATION seconds.
  3. Publish the contact point on /safety_obstacles in base_link frame with
     the moment-of-bump timestamp. Nav2's dedicated safety_obstacles layer
     (clearing=false) retains the mark; scan raytracing cannot clear it.

After the reverse the robot is stopped and waits for new instructions.

Marks have a TTL (default 60s). When any mark expires, we call Nav2's
clear_entirely services for both costmaps and drop the expired mark from
our internal tracking. The scan_obstacles layer immediately repopulates
from the next lidar tick; non-expired safety marks get re-published and
re-appear on the next node tick.
"""

import json

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool


# Contact positions in base_link frame (meters). Roomba 700 is ~34cm diameter.
HAZARD_POINTS_BASE_LINK = {
    'bump_left':         (0.17,  0.08, 0.0),
    'bump_right':        (0.17, -0.08, 0.0),
    'cliff_left':        (0.12,  0.13, 0.0),
    'cliff_front_left':  (0.16,  0.05, 0.0),
    'cliff_front_right': (0.16, -0.05, 0.0),
    'cliff_right':       (0.12, -0.13, 0.0),
}

# packet_id (str, matches roomba_node JSON keys) -> [(field, hazard_key), ...]
PACKET_FIELDS = {
    '7':  [('bump_left', 'bump_left'), ('bump_right', 'bump_right')],
    '9':  [('cliff_left', 'cliff_left')],
    '10': [('cliff_front_left', 'cliff_front_left')],
    '11': [('cliff_front_right', 'cliff_front_right')],
    '12': [('cliff_right', 'cliff_right')],
}


class BumpObstacleNode(Node):
    def __init__(self):
        super().__init__('bump_obstacle')

        self.declare_parameter('reverse_speed', 0.15)
        self.declare_parameter('reverse_duration', 1.0)
        self.declare_parameter('emergency_cooldown', 2.0)
        self.declare_parameter('mark_ttl_seconds', 60.0)
        # Bump debounce — the bumper stays deflected through a surface
        # transition (e.g. hard floor onto plush carpet) for up to ~1s as
        # the robot pushes through the pile. Require the bump to persist at
        # least this long before treating it as a real obstacle contact.
        # At 0.15 m/s a 1.5s window = ~22cm of push, well within the
        # bumper's mechanical travel.
        self.declare_parameter('bump_debounce_seconds', 1.5)
        # Cliff detection — the IR sensors on the underside of the robot
        # also false-fire when the front pitches up crossing a threshold
        # (carpet edge, door sill). Disable in environments with no real
        # cliffs; re-enable if the robot could roll off a stair/step.
        self.declare_parameter('cliff_detection_enabled', False)

        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.reverse_duration = float(self.get_parameter('reverse_duration').value)
        self.emergency_cooldown = float(self.get_parameter('emergency_cooldown').value)
        self.mark_ttl_ns = int(
            float(self.get_parameter('mark_ttl_seconds').value) * 1e9
        )
        self.bump_debounce_ns = int(
            float(self.get_parameter('bump_debounce_seconds').value) * 1e9
        )
        self.cliff_detection_enabled = bool(
            self.get_parameter('cliff_detection_enabled').value
        )

        # Which bump hazard keys require debouncing (cliffs do not).
        self._bump_keys = {'bump_left', 'bump_right'}
        # hazard_key -> time (ns) the bump went True, or absent if currently False.
        self._bump_rising_ns: dict[str, int] = {}

        # hazard_key -> {'point', 'stamp' (msg), 'ttl_expiry_ns'}
        self._snapshots: dict[str, dict] = {}

        self._reverse_until_ns = 0
        self._last_emergency_ns = 0
        # Allow callers (e.g., docking sequence) to suppress emergency
        # behavior. Sensor messages are still received but ignored.
        self._enabled = True

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/safety_obstacles', 10)
        # Emitted whenever the emergency sequence actually fires (after
        # debounce, cooldown, etc.) — consumers (reasoning_node) use this
        # to explain to the LLM why a nav action was canceled.
        self.interrupt_pub = self.create_publisher(
            String, '/bump/interrupt_event', 10
        )

        self.sensor_sub = self.create_subscription(
            String, '/roomba/sensors', self._on_sensors, 10
        )

        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal'
        )

        self.set_enabled_srv = self.create_service(
            SetBool, '~/set_enabled', self._on_set_enabled
        )
        self.clear_local_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )

        # 20Hz — fast enough to dominate cmd_vel during the reverse window.
        self.tick_timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'bump_obstacle ready. mark_ttl={self.mark_ttl_ns / 1e9:.0f}s, '
            f'reverse={self.reverse_duration:.1f}s @ {self.reverse_speed:.2f} m/s, '
            f'bump_debounce={self.bump_debounce_ns / 1e9:.1f}s, '
            f'cliff={"on" if self.cliff_detection_enabled else "OFF"}'
        )

    # ---------- enable/disable ----------

    def _on_set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._enabled = bool(request.data)
        state = 'enabled' if self._enabled else 'DISABLED'
        self.get_logger().info(f'bump_obstacle {state}')
        response.success = True
        response.message = state
        return response

    # ---------- sensor handling ----------

    def _on_sensors(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        now = self.get_clock().now()
        now_ns = now.nanoseconds

        # Collect which hazard keys are firing right now.
        firing: set[str] = set()
        for packet_id, fields in PACKET_FIELDS.items():
            packet = data.get(packet_id)
            if not isinstance(packet, dict):
                continue
            for field_name, hazard_key in fields:
                if packet.get(field_name):
                    firing.add(hazard_key)

        # Cliffs: trigger immediately (safety-critical, no debounce) — but
        # only when enabled. False-fires on threshold transitions make this
        # worse than useless in environments without real cliffs.
        if self.cliff_detection_enabled:
            for key in firing:
                if key not in self._bump_keys:
                    self._on_hazard(key, now)

        # Bumps: debounce. Track rising edges; only dispatch to _on_hazard
        # once the bump has been continuously True for bump_debounce_ns.
        for key in self._bump_keys:
            if key in firing:
                if key not in self._bump_rising_ns:
                    # Rising edge — start timer, do NOT fire yet.
                    self._bump_rising_ns[key] = now_ns
                elif now_ns - self._bump_rising_ns[key] >= self.bump_debounce_ns:
                    # Sustained past the window — real bump, fire.
                    self._on_hazard(key, now)
            else:
                # Falling edge (or still idle) — clear the timer so a new
                # rising edge restarts debounce.
                self._bump_rising_ns.pop(key, None)

    def _on_hazard(self, key: str, now: Time) -> None:
        now_ns = now.nanoseconds
        # (Re)snapshot on every firing so the mark has a fresh stamp + TTL.
        self._snapshots[key] = {
            'point': HAZARD_POINTS_BASE_LINK[key],
            'stamp': now.to_msg(),
            'ttl_expiry_ns': now_ns + self.mark_ttl_ns,
        }

        if now_ns < self._reverse_until_ns:
            return  # already reversing
        if now_ns - self._last_emergency_ns < int(self.emergency_cooldown * 1e9):
            return  # within cooldown

        self._last_emergency_ns = now_ns
        self._reverse_until_ns = now_ns + int(self.reverse_duration * 1e9)
        self.get_logger().warn(
            f'{key} fired -> cancel nav, reverse {self.reverse_duration:.1f}s, mark'
        )
        self._cancel_nav_goals()

        # Publish an interrupt event so upstream reasoning can explain why
        # whatever action was in flight got canceled.
        event = String()
        event.data = json.dumps({
            'hazard': key,
            'contact_point_base_link': list(HAZARD_POINTS_BASE_LINK[key]),
            'timestamp_ns': now_ns,
        })
        self.interrupt_pub.publish(event)

    def _cancel_nav_goals(self) -> None:
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn('cancel_goal service not ready; skipping cancel')
            return
        # Empty request (zero goal_id + zero stamp) cancels all active goals.
        self.cancel_client.call_async(CancelGoal.Request())

    # ---------- periodic output ----------

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        # 1) Reverse cmd_vel during the emergency window.
        if now_ns < self._reverse_until_ns:
            twist = Twist()
            twist.linear.x = -self.reverse_speed
            self.cmd_vel_pub.publish(twist)
        elif 0 < self._reverse_until_ns <= now_ns:
            self.cmd_vel_pub.publish(Twist())  # stop
            self._reverse_until_ns = 0

        # 2) Drop TTL-expired marks; clear costmaps if any expired.
        expired = [
            k for k, s in self._snapshots.items()
            if s['ttl_expiry_ns'] < now_ns
        ]
        if expired:
            for k in expired:
                del self._snapshots[k]
            self.get_logger().info(
                f'marks expired ({len(expired)}): {expired} — clearing costmaps'
            )
            self._clear_costmaps()

        # 3) Publish active snapshots so they (stay) marked in the costmap.
        if self._snapshots:
            self._publish_snapshots()

    def _publish_snapshots(self) -> None:
        # Each snapshot keeps its original stamp so Nav2 uses the TF at the
        # bump instant. Group by stamp and publish one cloud per stamp.
        by_stamp: dict[tuple[int, int], list] = {}
        for snap in self._snapshots.values():
            key = (snap['stamp'].sec, snap['stamp'].nanosec)
            by_stamp.setdefault(key, []).append(snap['point'])

        for (sec, nsec), points in by_stamp.items():
            header = Header()
            header.stamp.sec = sec
            header.stamp.nanosec = nsec
            header.frame_id = 'base_link'
            cloud = point_cloud2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(cloud)

    def _clear_costmaps(self) -> None:
        if self.clear_local_client.service_is_ready():
            self.clear_local_client.call_async(ClearEntireCostmap.Request())
        if self.clear_global_client.service_is_ready():
            self.clear_global_client.call_async(ClearEntireCostmap.Request())


def main(args=None):
    rclpy.init(args=args)
    node = BumpObstacleNode()
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
