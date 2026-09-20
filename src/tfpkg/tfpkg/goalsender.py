#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from typing import List, Tuple

class LocalSteerEscapeBroadcaster(Node):
    def __init__(self):
        super().__init__('escape_target_broadcaster')

        # 1. Configuration & Thresholds
        self.map_frame = 'map'
        self.robot_frame = 'our_bot/base_link'
        self.enemy_frame = 'enemy_bot/base_link'
        self.target_frame = 'dynamic_nav_target'
        
        # 2. TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3. Main Control Loop (Runs at 10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # --- SPEED OPTIMIZATION 1: Precompute Angles ---
        self.sample_radii = [0.8, 1.2, 1.8, 2.0] # meters
        self.angle_steps = []
        for deg in range(0, 360, 8):       # 15-degree slices
            rad = math.radians(deg)
            self.angle_steps.append((math.cos(rad), math.sin(rad)))

        # --- Map Caching Setup ---
        self.map_data = None
        self.map_res = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.MAX_SAFE_COST = 80  # Adjust based on inflation radius

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 'our_bot/global_costmap/costmap', self.costmap_callback, 10
        )

        # --- Hysteresis (Anti-Jitter Memory) ---
        self.last_target_x = None
        self.last_target_y = None
        self.MIN_UPDATE_DIST = 0.15  # Deadband threshold (meters)

        self.get_logger().info("Evasion Planner initialized. Monitoring threat zones...")

    def get_yaw_from_quaternion(self, q):
        """Fast conversion from geometry_msgs Quaternion to Euler Yaw."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def costmap_callback(self, msg: OccupancyGrid):
        """SPEED OPTIMIZATION 2: Flatten map info into class variables."""
        self.map_data = msg.data
        self.map_res = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def evaluate_threat_state(self, rel_x: float, rel_y: float):
        """Phase 1: Dynamic Danger Zones using relative coordinates."""
        distance = math.hypot(rel_x, rel_y)
        baseline_escape_angle = math.atan2(rel_y, rel_x)
        relative_angle_deg = abs(math.degrees(baseline_escape_angle))

        # 1. Define the angle zones
        in_front_cone = relative_angle_deg <= 67.5       # The main 135° front camera
        in_back_cone = relative_angle_deg >= 135.0       # The 90° rear zone
        
        # 2. Determine the danger radius based strictly on WHERE we are
        if in_front_cone:
            current_threshold = 2.8  # Very dangerous in front!
        elif in_back_cone:
            current_threshold = 0.0  # Safe in the back.
        else:
            current_threshold = 2.0  # Horizontal/Perpendicular flanks.
            
        # 3. Master Distance Check
        if distance > current_threshold:
            return False, distance, None
            
        # 4. We are too close. Run!
        self.get_logger().warn(f"EVADE! Dist: {distance:.2f}m <= {current_threshold}m zone threshold")
        return True, distance, baseline_escape_angle

    def get_cell_cost(self, x: float, y: float) -> int:
        """Fast 1D array lookup for costmap cost."""
        if self.map_data is None:
            return 255 # Assume blocked if no map arrived yet
            
        col = int((x - self.map_origin_x) / self.map_res)
        row = int((y - self.map_origin_y) / self.map_res)

        if col < 0 or col >= self.map_width or row < 0 or row >= self.map_height:
            return 255
            
        return self.map_data[row * self.map_width + col]

    def is_path_clear(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        """Fast raycast: Walks the line in 5cm steps checking for walls."""
        dx = end_x - start_x
        dy = end_y - start_y
        dist = math.hypot(dx, dy)
        
        steps = max(2, int(dist / 0.05))
        step_x = dx / steps
        step_y = dy / steps
        
        curr_x, curr_y = start_x, start_y
        for _ in range(steps):
            curr_x += step_x
            curr_y += step_y
            
            cost = self.get_cell_cost(curr_x, curr_y)
            if cost > self.MAX_SAFE_COST or cost == -1:
                return False # Hit a wall or unknown space
                
        return True

    def get_safe_escape_points(self, robot_x: float, robot_y: float) -> list:
            safe_points = []
            
            for r in self.sample_radii:
                for cos_val, sin_val in self.angle_steps:
                    cand_x = robot_x + (r * cos_val)
                    cand_y = robot_y + (r * sin_val)
                    
                    if self.is_path_clear(robot_x, robot_y, cand_x, cand_y):
                        # NEW: Get the exact cost of the final landing spot
                        final_cost = self.get_cell_cost(cand_x, cand_y)
                        
                        # Add final_cost to our tuple so Phase 4 can use it
                        safe_points.append((cand_x, cand_y, r, final_cost))
                        
            return safe_points

    def get_best_escape_target(self, safe_points, our_x, our_y, our_yaw, enemy_x, enemy_y):
        best_point = None
        highest_score = -float('inf')

        # NEW: Unpack the 'cost' variable we added
        for cand_x, cand_y, r, cost in safe_points:
            score = 0.0

            # 1. Threat distance reward
            dist_to_enemy = math.hypot(cand_x - enemy_x, cand_y - enemy_y)
            score += dist_to_enemy * 10.0

            # 2. Momentum reward
            score += r * 2.0

            # 3. Kinematic Penalty 
            angle_to_cand = math.atan2(cand_y - our_y, cand_x - our_x)
            turn_diff = abs(math.atan2(math.sin(angle_to_cand - our_yaw), 
                                       math.cos(angle_to_cand - our_yaw)))
            score -= turn_diff * 2.0 

            # ==========================================
            # 4. NEW: COSTMAP PENALTY (Keeps us off the walls!)
            # ==========================================
            # If the point is in open space (cost 0), it loses no points.
            # If the point is near a wall (e.g., cost 100), it loses a massive 30 points.
            score -= cost * 0.3 

            if score > highest_score:
                highest_score = score
                best_point = (cand_x, cand_y, angle_to_cand)

        return best_point

    def control_loop(self):
        try:
            # 1. Look up global positions
            tf_global = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
            tf_enemy_global = self.tf_buffer.lookup_transform(
                self.map_frame, self.enemy_frame, rclpy.time.Time()
            )
            
            # 2. Look up relative position
            tf_relative = self.tf_buffer.lookup_transform(
                self.enemy_frame, self.robot_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return

        # Extract coordinates
        global_x = tf_global.transform.translation.x
        global_y = tf_global.transform.translation.y
        our_yaw = self.get_yaw_from_quaternion(tf_global.transform.rotation)
        
        enemy_global_x = tf_enemy_global.transform.translation.x
        enemy_global_y = tf_enemy_global.transform.translation.y

        rel_x = tf_relative.transform.translation.x
        rel_y = tf_relative.transform.translation.y

        # ==========================================
        # PHASE 1: THREAT EVALUATION
        # ==========================================
        in_danger, distance, _ = self.evaluate_threat_state(rel_x, rel_y)
        
        if not in_danger:
            self.last_target_x = None  # Reset hysteresis when safe
            return 

        # ==========================================
        # PHASE 2 & 3: GENERATE SAFE CANDIDATE POINTS
        # ==========================================
        safe_points = self.get_safe_escape_points(global_x, global_y)
        
        if not safe_points:
            self.get_logger().warn("TRAPPED! No safe escape paths found!")
            return
            
        # ==========================================
        # PHASE 4: SCORE POINTS AND BROADCAST
        # ==========================================
        target_x, target_y, target_yaw = self.get_best_escape_target(
            safe_points, global_x, global_y, our_yaw, enemy_global_x, enemy_global_y
        )
        
        # Deadband Filter: Ignore updates if target shifted by less than 15cm
        if self.last_target_x is not None:
            shift = math.hypot(target_x - self.last_target_x, target_y - self.last_target_y)
            if shift < self.MIN_UPDATE_DIST:
                return  

        self.last_target_x = target_x
        self.last_target_y = target_y

        # Broadcast the winning point to TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.target_frame

        t.transform.translation.x = target_x
        t.transform.translation.y = target_y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(target_yaw / 2.0)
        t.transform.rotation.w = math.cos(target_yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Evading! Target broadcasted at X:{target_x:.2f}, Y:{target_y:.2f}")


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