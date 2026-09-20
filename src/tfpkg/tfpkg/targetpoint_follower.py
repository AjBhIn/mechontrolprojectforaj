#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import tf2_ros
import tf2_geometry_msgs


class ShortGoalStalkerNode(Node):
    def __init__(self):
        super().__init__('short_goal_stalker_node')

        # ==========================================
        # 1. MULTITHREADING CALLBACK GROUP
        # ==========================================
        self.cb_group = ReentrantCallbackGroup()

        # ==========================================
        # 2. PARAMETERS (Short Goal Config)
        # ==========================================
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pursuer_frame', 'our_bot/base_link')
        self.declare_parameter('enemy_frame', 'enemy_bot/base_link')
        self.declare_parameter('target_frame', 'enemy_bot/target_point')
        self.declare_parameter('costmap_topic', '/our_bot/global_costmap/costmap_raw')
        self.declare_parameter('action_name', 'our_bot/navigate_to_pose')
        self.declare_parameter('cmd_vel_topic', '/our_bot/cmd_vel')

        # Short Goal & Evasion Parameters
        self.declare_parameter('short_goal_dist', 0.8)         # Maximum step-ahead goal distance (meters)
        self.declare_parameter('evade_trigger_dist', 2.0)      # Head-on FOV trigger distance (meters)
        self.declare_parameter('evade_clear_dist', 2.8)        # Distance to clear evasion state (meters)
        self.declare_parameter('min_dist_threshold', 0.25)     # Resend threshold (meters) for rapid updates

        self.map_frame = self.get_parameter('map_frame').value
        self.pursuer_frame = self.get_parameter('pursuer_frame').value
        self.enemy_frame = self.get_parameter('enemy_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.action_name = self.get_parameter('action_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.short_goal_dist = float(self.get_parameter('short_goal_dist').value)
        self.evade_trigger_dist = float(self.get_parameter('evade_trigger_dist').value)
        self.evade_clear_dist = float(self.get_parameter('evade_clear_dist').value)
        self.min_dist_threshold = float(self.get_parameter('min_dist_threshold').value)

        # State Variables
        self.state = 'CHASING'
        self.last_sent_pose = None
        self.active_goal_handle = None
        self.costmap = None

        # ==========================================
        # 3. ROS INTERFACES 
        # ==========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self.costmap_callback, 10, callback_group=self.cb_group
        )
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.action_name, callback_group=self.cb_group)
        self.timer = self.create_timer(0.1, self.control_loop, callback_group=self.cb_group)  # 10 Hz Loop

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg

    def emergency_brake(self):
        self.cmd_vel_pub.publish(Twist())

    def cancel_active_goal(self):
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

    # ==========================================
    # 4. COSTMAP OBSTACLE PROTECTION
    # ==========================================
    def is_pose_safe(self, x: float, y: float) -> bool:
        if self.costmap is None:
            return True
        info = self.costmap.info
        gx = int((x - info.origin.position.x) / info.resolution) # getting the distance
        gy = int((y - info.origin.position.y) / info.resolution)

        if 0 <= gx < info.width and 0 <= gy < info.height:
            cost = self.costmap.data[gy * info.width + gx]
            return cost < 180  # Stay clear of lethal obstacles and high inflation
        return False

    def project_to_free_space(self, start_x: float, start_y: float, target_x: float, target_y: float):
        """Pulls target pose toward pursuer if short goal touches an obstacle/inflation."""
        if self.is_pose_safe(target_x, target_y):
            return target_x, target_y

        dist = math.hypot(target_x - start_x, target_y - start_y)
        if dist < 0.05:
            return start_x, start_y

        steps = 10
        for i in range(1, steps):
            alpha = 1.0 - (i / float(steps))
            px = start_x + alpha * (target_x - start_x)
            py = start_y + alpha * (target_y - start_y)
            if self.is_pose_safe(px, py):
                return px, py

        return start_x, start_y

    # ==========================================
    # 5. TF TRANSFORM LOOKUPS
    # ==========================================
    def get_pose_in_map(self, frame_id: str):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.03)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation

            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except tf2_ros.TransformException:
            return None, None, None

    def get_pursuer_in_enemy_frame(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.enemy_frame, self.pursuer_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.03)
            )
            return trans.transform.translation.x, trans.transform.translation.y
        except tf2_ros.TransformException:
            return None, None

    # ==========================================
    # 6. SHORT-GOAL CLAMPING & STATE MACHINE
    # ==========================================
    def update_state(self, x_e: float, y_e: float):
        dist = math.hypot(x_e, y_e)
        in_front_arc = x_e > -0.2  # Enemy facing toward pursuer

        if self.state == 'CHASING':
            if in_front_arc and dist < self.evade_trigger_dist:
                self.get_logger().warn('Head-On FOV Detected! Switching to EVADING.')
                self.state = 'EVADING'
                self.emergency_brake()
                self.cancel_active_goal()
                self.last_sent_pose = None
        elif self.state == 'EVADING':
            if dist > self.evade_clear_dist or x_e < -0.4:
                self.get_logger().info('Clear of FOV. Resuming CHASING.')
                self.state = 'CHASING'
                self.cancel_active_goal()
                self.last_sent_pose = None

    def compute_short_goal(self) -> PoseStamped:
        x_e, y_e = self.get_pursuer_in_enemy_frame()
        p_x, p_y, p_yaw = self.get_pose_in_map(self.pursuer_frame)
        e_x, e_y, _ = self.get_pose_in_map(self.enemy_frame)

        if x_e is None or p_x is None or e_x is None:
            return None

        self.update_state(x_e, y_e)

        # 1. EVADING MODE: Project short goal backward away from enemy
        if self.state == 'EVADING':
            dx_away = p_x - e_x
            dy_away = p_y - e_y
            dist_away = math.hypot(dx_away, dy_away)

            if dist_away > 0.01:
                target_x = p_x + (dx_away / dist_away) * self.short_goal_dist
                target_y = p_y + (dy_away / dist_away) * self.short_goal_dist
            else:
                target_x = p_x - self.short_goal_dist * math.cos(p_yaw)
                target_y = p_y - self.short_goal_dist * math.sin(p_yaw)

            target_yaw = math.atan2(dy_away, dx_away)

        # 2. CHASING MODE: Project short goal toward target_frame TF
        else:
            t_x, t_y, t_yaw = self.get_pose_in_map(self.target_frame)
            if t_x is None:
                return None

            dx = t_x - p_x
            dy = t_y - p_y
            dist_to_target = math.hypot(dx, dy)

            # Short Goal Clamping: Cap goal distance to short_goal_dist (0.8m)
            if dist_to_target > self.short_goal_dist:
                ratio = self.short_goal_dist / dist_to_target
                target_x = p_x + dx * ratio
                target_y = p_y + dy * ratio
            else:
                target_x = t_x
                target_y = t_y

            target_yaw = t_yaw

        # Ensure short goal coordinate sits on free costmap space
        safe_x, safe_y = self.project_to_free_space(p_x, p_y, target_x, target_y)

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = safe_x
        goal.pose.position.y = safe_y
        goal.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.orientation.w = math.cos(target_yaw / 2.0)

        return goal

    # ==========================================
    # 7. CONTROL LOOP & THROTTLING
    # ==========================================
    def should_resend_goal(self, new_pose: PoseStamped) -> bool:
        if self.last_sent_pose is None:
            return True

        dx = new_pose.pose.position.x - self.last_sent_pose.pose.position.x
        dy = new_pose.pose.position.y - self.last_sent_pose.pose.position.y
        return math.hypot(dx, dy) >= self.min_dist_threshold

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.active_goal_handle = goal_handle

    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.02):
            return

        candidate_goal = self.compute_short_goal()
        if candidate_goal is None:
            return

        if self.should_resend_goal(candidate_goal):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = candidate_goal

            send_future = self.nav_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self.goal_response_callback)
            self.last_sent_pose = candidate_goal


# ==========================================
# 8. EXECUTION ENTRYPOINT
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = ShortGoalStalkerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()