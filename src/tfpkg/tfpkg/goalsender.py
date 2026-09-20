#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid


class LocalSteerEscapeBroadcaster(Node):
    def __init__(self):
        super().__init__('escape_target_broadcaster')

        # 1. Configuration & Thresholds
        self.map_frame = 'map'
        self.robot_frame = 'our_bot/base_link'
        self.enemy_frame = 'enemy_bot/base_link'
        
        self.danger_threshold = 2.5  # meters
        
        # Attack Cones (in degrees)
        self.front_attack_angle = 135.0  # Front threat cone
        self.back_attack_angle = 90.0    # Rear threat cone (adjustable)

        # 2. TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. Main Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Phase 1: Directional Threat Monitor initialized.")

    def get_yaw_from_quaternion(self, q) -> float:
        """Converts a ROS geometry_msgs Quaternion to a Euler Yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        try:
            # 1. Look up our global position (needed for Phase 2 later)
            tf_global = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
            
            # Look up our position relative to the enemy!
            tf_relative = self.tf_buffer.lookup_transform(
                self.enemy_frame, self.robot_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return

        # Where are we globally? (For drawing the escape routes later)
        global_x = tf_global.transform.translation.x
        global_y = tf_global.transform.translation.y
        
        # Where are we locally to the enemy? (For the threat check)
        rel_x = tf_relative.transform.translation.x
        rel_y = tf_relative.transform.translation.y

        # Run the new, simplified threat check
        in_danger, distance, global_escape_angle = self.evaluate_threat_state(
            rel_x, rel_y
        )
        


    def evaluate_threat_state(self, rel_x: float, rel_y: float):
        """
        Evaluates if we are inside the enemy's front 1.8m semi-circle.
        Uses relative coordinates where the enemy is at (0,0) facing +X.
        """
        distance = math.hypot(rel_x, rel_y)
        baseline_escape_angle = math.atan2(rel_y, rel_x)
        relative_angle_deg = abs(math.degrees(baseline_escape_angle))
        
        # 1. Define the angle zones
        in_front_cone = relative_angle_deg <= 67.5       # The main 135° front camera
        in_dead_center = relative_angle_deg <= 23.0      # Narrow 46° cone directly in front
        in_back_cone = relative_angle_deg >= 135.0       # The 90° rear zone
        
        # 2. Determine the danger radius based strictly on WHERE we are
        if in_front_cone:
            current_threshold = 2.8  # Very dangerous in front!
        elif in_back_cone:
            current_threshold = 0.0  # (Assuming you want the back to always be safe)
        else:
            # If we aren't in the front or the back, we are on the sides (flanks).
            current_threshold = 2.0  # Horizontal danger radius
            
        # 3. The Single Master Distance Check
        if distance > current_threshold:
            self.get_logger().info(f"SAFE! Dist: {distance:.2f}m > Threshold: {current_threshold}m")
            return False, distance, None
            
        # 4. If we survived the check above, we are too close. Run!
        self.get_logger().warn(f"NOT SAFE! Dist: {distance:.2f}m <= Threshold: {current_threshold}m")
        return True, distance, baseline_escape_angle



   

def main(args=None):
    rclpy.init(args=args)
    node = LocalSteerEscapeBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()