#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from typing import List, Tuple, Optional

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
        for deg in range(0, 360, 8):       # 8-degree slices
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

        # --- FEATURE ADDITION: Memory Cache Bank ---
        # Stores top-ranked points from previous execution cycles to save CPU cycles
        self.memory_bank: List[Tuple[float, float, float]] = []

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
        """
        Fast 1D array lookup with Map Boundary Squeezing/Clamping.
        Guarantees floating point map coordinates outside boundaries
        are squeezed into valid 2D array cells without index errors or 255 traps.
        """
        if self.map_data is None:
            return 255 # Assume blocked if no map arrived yet
            
        # Calculate raw float grid indices
        raw_col = int((x - self.map_origin_x) / self.map_res)
        raw_row = int((y - self.map_origin_y) / self.map_res)

        # MAP SQUEEZING / CLAMPING: Restrict indices inside [0, width-1] and [0, height-1]
        col = max(0, min(raw_col, self.map_width - 1))
        row = max(0, min(raw_row, self.map_height - 1))

        # Direct 1D array access
        cost = self.map_data[row * self.map_width + col]
        return 255 if cost == -1 else cost

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

    # =========================================================================
    # FEATURE 1: MEMORY CACHE BANK (Fast-Path Check)
    # =========================================================================
    def check_memory_bank(self, our_x: float, our_y: float, enemy_x: float, enemy_y: float) -> List[Tuple[float, float, float]]:
        """
        Validates cached escape points from previous loops.
        If a stored point remains safe and keeps us moving away from the enemy,
        we can re-use it to skip full grid generation and save CPU cycles.
        """
        if not self.memory_bank:
            return []

        still_valid_points = []
        current_dist_to_enemy = math.hypot(our_x - enemy_x, our_y - enemy_y)

        for mx, my, myaw in self.memory_bank:
            # 1. Distance Test: Does this stored point still increase distance from enemy?
            mem_dist_to_enemy = math.hypot(mx - enemy_x, my - enemy_y)
            if mem_dist_to_enemy <= (current_dist_to_enemy + 0.3):
                continue

            # 2. Raycast Test: Is the path from our current spot to this stored point still clear?
            if not self.is_path_clear(our_x, our_y, mx, my):
                continue

            # 3. Landing Zone Test: Is the destination point still free of dynamic obstacles?
            if self.get_cell_cost(mx, my) > self.MAX_SAFE_COST:
                continue

            still_valid_points.append((mx, my, myaw))

        return still_valid_points

    # =========================================================================
    # FEATURE 2: ADAPTIVE APF WEIGHTS (Context-Aware Multipliers)
    # =========================================================================
    def get_adaptive_weights(self, current_cost: int, dist_to_enemy: float) -> dict:
        """
        Dynamically adjusts scoring weights depending on spatial context.
        Prioritizes high speed in open areas, wall avoidance in tight areas,
        and maximum raw displacement when panic-close to the enemy.
        """
        # Default baseline weights
        w_dist = 10.0
        w_speed = 2.0
        w_turn = 2.0
        w_cost = 0.3

        # Tight Space Adjustments: Near obstacles/narrow corridors
        if current_cost > 30:
            w_cost = 1.0     # Strictly penalize hugging walls
            w_turn = 0.5     # Allow sharper turns if it helps us get away from walls

        # Emergency Adjustments: Threat is extremely close
        if dist_to_enemy < 1.2:
            w_dist = 20.0    # Put maximum priority on sheer escape distance
            w_speed = 4.0    # Encourage larger strides/steps

        return {
            'w_dist': w_dist,
            'w_speed': w_speed,
            'w_turn': w_turn,
            'w_cost': w_cost
        }

    # =========================================================================
    # FEATURE 3: MINI-MCTS 1-STEP LOOKAHEAD (Dead-End Detection)
    # =========================================================================
    def evaluate_future_options(self, cand_x: float, cand_y: float) -> float:
        """
        Performs a secondary, lightweight mini-sweep from the candidate point.
        Measures how many valid forward escape branches remain open, preventing
        the planner from steering into corners or dead-ends.
        """
        future_clear_branches = 0
        probe_radius = 0.5

        # Sub-sample every 4th angle (32° steps) to keep CPU overhead negligible
        for cos_val, sin_val in self.angle_steps[::4]:
            next_x = cand_x + (probe_radius * cos_val)
            next_y = cand_y + (probe_radius * sin_val)

            if self.is_path_clear(cand_x, cand_y, next_x, next_y):
                if self.get_cell_cost(next_x, next_y) <= self.MAX_SAFE_COST:
                    future_clear_branches += 1

        # Reward candidate points that preserve open escape routes
        return future_clear_branches * 1.5

    def get_safe_escape_points(self, robot_x: float, robot_y: float) -> list:
        safe_points = []
        
        for r in self.sample_radii:
            for cos_val, sin_val in self.angle_steps:
                cand_x = robot_x + (r * cos_val)
                cand_y = robot_y + (r * sin_val)
                
                if self.is_path_clear(robot_x, robot_y, cand_x, cand_y):
                    # Get the exact cost of the final landing spot
                    final_cost = self.get_cell_cost(cand_x, cand_y)
                    
                    # Add final_cost to our tuple so Phase 4 can use it
                    safe_points.append((cand_x, cand_y, r, final_cost))
                    
        return safe_points

    def get_best_escape_target(self, safe_points, our_x, our_y, our_yaw, enemy_x, enemy_y):
        best_point = None
        highest_score = -float('inf')
        all_scored_points = []

        our_current_cost = self.get_cell_cost(our_x, our_y)
        dist_to_enemy = math.hypot(our_x - enemy_x, our_y - enemy_y)

        # 1. Fetch dynamic AI weights based on our current situation
        weights = self.get_adaptive_weights(our_current_cost, dist_to_enemy)

        for cand_x, cand_y, r, cost in safe_points:
            score = 0.0

            # 1. Threat distance reward (using adaptive weight)
            cand_dist_to_enemy = math.hypot(cand_x - enemy_x, cand_y - enemy_y)
            score += cand_dist_to_enemy * weights['w_dist']

            # 2. Momentum reward (using adaptive weight)
            score += r * weights['w_speed']

            # 3. Kinematic Penalty (using adaptive weight)
            angle_to_cand = math.atan2(cand_y - our_y, cand_x - our_x)
            turn_diff = abs(math.atan2(math.sin(angle_to_cand - our_yaw), 
                                       math.cos(angle_to_cand - our_yaw)))
            score -= turn_diff * weights['w_turn']

            # 4. Costmap Penalty (using adaptive weight)
            score -= cost * weights['w_cost']

            # 5. Mini-MCTS: Lookahead branch score
            score += self.evaluate_future_options(cand_x, cand_y)

            # Store for memory bank update
            all_scored_points.append((score, cand_x, cand_y, angle_to_cand))

            if score > highest_score:
                highest_score = score
                best_point = (cand_x, cand_y, angle_to_cand)

        # Update the memory bank with top-ranked candidates for subsequent loops
        all_scored_points.sort(key=lambda item: item[0], reverse=True)
        self.memory_bank = [(item[1], item[2], item[3]) for item in all_scored_points[:3]]

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
            self.memory_bank.clear()   # Clear cache when out of threat state
            return 

        # ==========================================
        # PHASE 1.5: MEMORY FAST-PATH CHECK
        # ==========================================
        valid_cached_points = self.check_memory_bank(global_x, global_y, enemy_global_x, enemy_global_y)
        
        if valid_cached_points:
            # Fast Path: Re-use validated point, skipping full grid generation
            target_x, target_y, target_yaw = valid_cached_points[0]
            self.get_logger().debug("Using validated target from Memory Bank (Fast-Path).")
        else:
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
            best_target = self.get_best_escape_target(
                safe_points, global_x, global_y, our_yaw, enemy_global_x, enemy_global_y
            )
            
            if best_target is None:
                return

            target_x, target_y, target_yaw = best_target
        
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