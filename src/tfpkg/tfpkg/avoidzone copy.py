#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf2_ros
import tf2_geometry_msgs  # Registers TF2 conversions for PoseStamped


class SmoothFlankerFollower(Node):
    def __init__(self):
        super().__init__('smooth_flanker_follower')

        # 1. Initialize TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2. Frame & Geometry Configuration
        self.map_frame = 'map'
        self.enemy_frame = 'enemy_bot/base_link'
        self.target_frame = 'enemy_bot/target_point'
        self.pursuer_frame = 'our_bot/base_link'

        self.flank_radius = 1.8         # Distance (meters) to clear front camera
        self.flank_angle_deg = 115.0    # Angle outside front 180° camera zone (±90°)

        # 3. Track previous target pose to avoid goal spamming
        self.last_goal_x = 0.0
        self.last_goal_y = 0.0
        self.movement_threshold = 0.3   # Meters enemy must move before updating path

    def is_pursuer_on_left(self) -> bool:
        """Determines if our_bot is on the enemy's left (+Y) or right (-Y) side."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.enemy_frame,
                self.pursuer_frame,
                rclpy.time.Time()  # Time() uses latest available transform
            )
            return trans.transform.translation.y >= 0.0
        except tf2_ros.TransformException:
            return True  # Default to left side if lookup fails

    def get_flank_pose_in_map(self) -> PoseStamped:
        """Calculates side detour waypoint outside camera cone in map frame."""
        side_sign = 1.0 if self.is_pursuer_on_left() else -1.0
        angle_rad = math.radians(self.flank_angle_deg * side_sign)

        local_pose = PoseStamped()
        local_pose.header.frame_id = self.enemy_frame
        # Stamp with Time() (zero timestamp) to fetch latest transform
        local_pose.header.stamp = rclpy.time.Time().to_msg()
        
        local_pose.pose.position.x = self.flank_radius * math.cos(angle_rad)
        local_pose.pose.position.y = self.flank_radius * math.sin(angle_rad)
        local_pose.pose.orientation.z = math.sin(angle_rad / 2.0)
        local_pose.pose.orientation.w = math.cos(angle_rad / 2.0)

        try:
            return self.tf_buffer.transform(
                local_pose, 
                self.map_frame, 
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Flank TF transform failed: {ex}")
            return None

    def get_target_pose_in_map(self) -> PoseStamped:
        """Converts origin of enemy_bot/target_point into map frame."""
        target_local = PoseStamped()
        target_local.header.frame_id = self.target_frame
        # Stamp with Time() (zero timestamp) to fetch latest transform
        target_local.header.stamp = rclpy.time.Time().to_msg()
        target_local.pose.orientation.w = 1.0

        try:
            return self.tf_buffer.transform(
                target_local, 
                self.map_frame, 
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Target TF transform failed: {ex}")
            return None

    def check_and_send_trajectory(self, navigator: BasicNavigator):
        # A. Verify transform availability
        if not self.tf_buffer.can_transform(self.map_frame, self.target_frame, rclpy.time.Time()):
            self.get_logger().info("Waiting for transform from 'map' to 'enemy_bot/target_point'...", throttle_duration_sec=5.0)
            return

        # B. Get target kill pose
        target_pose = self.get_target_pose_in_map()
        if target_pose is None:
            return

        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y

        # C. Movement threshold check to prevent path re-planning spam
        dist_moved = math.sqrt((target_x - self.last_goal_x)**2 + (target_y - self.last_goal_y)**2)
        if dist_moved < self.movement_threshold:
            return

        # D. Get side detour waypoint pose
        flank_pose = self.get_flank_pose_in_map()
        if flank_pose is None:
            return

        # E. Send smooth multi-pose trajectory [Side Flank -> Target Point]
        self.get_logger().info(f"Dispatching smooth trajectory to Target X={target_x:.2f}, Y={target_y:.2f}")
        navigator.goThroughPoses([flank_pose, target_pose])

        # Update last sent position
        self.last_goal_x = target_x
        self.last_goal_y = target_y


def main(args=None):
    rclpy.init(args=args)

    node = SmoothFlankerFollower()
    navigator = BasicNavigator(namespace='our_bot')

    # Wait until Nav2 lifecycle nodes are active
    navigator.waitUntilNav2Active()

    # Execution loop
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        node.check_and_send_trajectory(navigator)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()