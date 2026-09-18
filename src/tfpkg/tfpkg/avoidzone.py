#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import tf2_ros
import tf2_geometry_msgs  # Registers TF2 conversions for PoseStamped


class DynamicFlankerFollower(Node):
    def __init__(self):
        super().__init__('dynamic_flanker_follower')

        # 1. TF Buffer & Listener Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2. Configurable Frame & Threshold Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('enemy_frame', 'enemy_bot/base_link')
        self.declare_parameter('target_frame', 'enemy_bot/target_point')
        self.declare_parameter('pursuer_frame', 'our_bot/base_link')
        self.declare_parameter('arc_radius', 2.0)            # Radius (m) around camera zone
        self.declare_parameter('dist_threshold', 0.2)        # Motion trigger delta (m)
        self.declare_parameter('yaw_threshold_deg', 15.0)   # Rotation trigger delta (deg)

        self.map_frame = self.get_parameter('map_frame').value
        self.enemy_frame = self.get_parameter('enemy_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.pursuer_frame = self.get_parameter('pursuer_frame').value
        self.arc_radius = self.get_parameter('arc_radius').value
        self.dist_threshold = self.get_parameter('dist_threshold').value
        self.yaw_threshold = math.radians(self.get_parameter('yaw_threshold_deg').value)

        # 3. Action Clients (Replaces BasicNavigator to fix Executor Spin error)
        self.client_through_poses = ActionClient(self, NavigateThroughPoses, '/our_bot/navigate_through_poses')
        self.client_to_pose = ActionClient(self, NavigateToPose, '/our_bot/navigate_to_pose')

        # 4. State Memory
        self.last_enemy_x = 0.0
        self.last_enemy_y = 0.0
        self.last_enemy_yaw = 0.0
        self.was_in_front = False
        self.goal_handle = None

        # 5. Control Loop Timer (5 Hz)
        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info("Dynamic Flanker Follower initialized with Async Action Clients.")

    # ==========================================================
    # HELPER FUNCTIONS: TF & GEOMETRY
    # ==========================================================
    def get_enemy_transform(self):
        """Fetches latest map-to-enemy transform and returns (x, y, yaw)."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.enemy_frame,
                rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            
            # Quaternion to Yaw conversion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except tf2_ros.TransformException:
            return None, None, None

    def check_pursuer_zone(self) -> tuple[bool, bool]:
        """
        Checks pursuer position in enemy's local frame.
        Returns: (is_in_front_180, is_on_left_side)
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.enemy_frame,
                self.pursuer_frame,
                rclpy.time.Time()
            )
            x_local = trans.transform.translation.x
            y_local = trans.transform.translation.y
            
            is_in_front = (x_local > 0.0)
            is_on_left = (y_local >= 0.0)
            return is_in_front, is_on_left
        except tf2_ros.TransformException:
            return True, True

    def create_enemy_local_pose(self, x: float, y: float, yaw_rad: float) -> PoseStamped:
        """Constructs PoseStamped relative to enemy_frame."""
        pose = PoseStamped()
        pose.header.frame_id = self.enemy_frame
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def transform_to_map(self, local_pose: PoseStamped) -> PoseStamped:
        """Transforms local pose into global map coordinates."""
        try:
            return self.tf_buffer.transform(
                local_pose, 
                self.map_frame, 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Map transform failed: {ex}")
            return None

    def calculate_trajectory(self) -> list[PoseStamped]:
        """Generates dynamic [W1, W2, Target] or [Target] path."""
        target_local = PoseStamped()
        target_local.header.frame_id = self.target_frame
        target_local.header.stamp = rclpy.time.Time().to_msg()
        target_local.pose.orientation.w = 1.0

        target_map = self.transform_to_map(target_local)
        if not target_map:
            return []

        is_in_front, is_on_left = self.check_pursuer_zone()

        if not is_in_front:
            return [target_map]

        side_sign = 1.0 if is_on_left else -1.0

        w1_local = self.create_enemy_local_pose(
            x=0.0, 
            y=side_sign * self.arc_radius, 
            yaw_rad=math.radians(90.0 * side_sign)
        )

        w2_x = -self.arc_radius * math.cos(math.radians(45.0))
        w2_y = side_sign * self.arc_radius * math.sin(math.radians(45.0))
        w2_local = self.create_enemy_local_pose(
            x=w2_x, 
            y=w2_y, 
            yaw_rad=math.radians(135.0 * side_sign)
        )

        w1_map = self.transform_to_map(w1_local)
        w2_map = self.transform_to_map(w2_local)

        if w1_map and w2_map:
            return [w1_map, w2_map, target_map]

        return [target_map]

    # ==========================================================
    # ASYNC ACTION GOAL DISPATCHING
    # ==========================================================
    def send_trajectory_async(self, waypoints: list[PoseStamped]):
        """Dispatches goals asynchronously without blocking the executor thread."""
        if len(waypoints) > 1:
            if not self.client_through_poses.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("Action server 'navigate_through_poses' not ready.")
                return
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = waypoints
            future = self.client_through_poses.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
        else:
            if not self.client_to_pose.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("Action server 'navigate_to_pose' not ready.")
                return
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = waypoints[0]
            future = self.client_to_pose.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.goal_handle = goal_handle

    # ==========================================================
    # 5-HZ CLOSED-LOOP CONTROL
    # ==========================================================
    def control_loop(self):
        curr_x, curr_y, curr_yaw = self.get_enemy_transform()
        if curr_x is None:
            return

        is_in_front, _ = self.check_pursuer_zone()

        dist_change = math.sqrt((curr_x - self.last_enemy_x)**2 + (curr_y - self.last_enemy_y)**2)
        yaw_change = abs(math.atan2(math.sin(curr_yaw - self.last_enemy_yaw), math.cos(curr_yaw - self.last_enemy_yaw)))

        emergency_flank_needed = (is_in_front and not self.was_in_front)
        motion_threshold_exceeded = (dist_change > self.dist_threshold or yaw_change > self.yaw_threshold)

        if motion_threshold_exceeded or emergency_flank_needed:
            waypoints = self.calculate_trajectory()
            if not waypoints:
                return

            self.get_logger().info(
                f"Enemy shifted (ΔPos: {dist_change:.2f}m, ΔYaw: {math.degrees(yaw_change):.1f}°). Updating path asynchronously."
            )

            # Send updated poses non-blockingly
            self.send_trajectory_async(waypoints)

            # Update state memory
            self.last_enemy_x = curr_x
            self.last_enemy_y = curr_y
            self.last_enemy_yaw = curr_yaw
            self.was_in_front = is_in_front


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFlankerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()