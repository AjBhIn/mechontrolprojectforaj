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


class GoalSetter(Node):
    def __init__(self):
        super().__init__('Goal_Setter')

        # ==========================================
        # 1. MULTITHREADING CALLBACK GROUP
        # ==========================================
        # self.cb_group = ReentrantCallbackGroup()

        # ==========================================
        # 2. PARAMETERS (Short Goal Config)
        # ==========================================
        # Delcaring the parameter
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pursuer_frame', 'our_bot/base_link')
        self.declare_parameter('enemy_frame', 'enemy_bot/base_link')
        self.declare_parameter('target_frame', 'enemy_bot/target_point')
        self.declare_parameter('costmap_topic', '/our_bot/global_costmap/costmap_raw')
        self.declare_parameter('action_name', 'our_bot/navigate_to_pose')
        self.declare_parameter('cmd_vel_topic', '/our_bot/cmd_vel')
        self.declare_parameter('escape_target_frame', 'dynamic_nav_target')

        # Getting parameter
        self.map_frame = self.get_parameter('map_frame').value
        self.pursuer_frame = self.get_parameter('pursuer_frame').value
        self.enemy_frame = self.get_parameter('enemy_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.action_name = self.get_parameter('action_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.escape_tf = self.get_parameter('escape_target_frame').value

        # ==========================================
        # 2. ROS 2 INTERFACES
        # ==========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)

        # Flag to prevent preemption loops
        self.is_goal_pending = False

        # Check for escape target at 1 Hz
        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info(f"Escape Nav Controller initialized. Listening for '{self.escape_tf}'")

    # ==========================================
    # 3. TF TARGET LOOKUP
    # ==========================================
    def get_escape_pose_in_map(self) -> PoseStamped:
        """Looks up dynamic_nav_target in map frame and converts it to PoseStamped."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.escape_tf,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame

            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = 0.0

            pose.pose.orientation = tf.transform.rotation
            return pose

        except tf2_ros.TransformException:
            # Broadcast node is not currently publishing dynamic_nav_target (robot is safe)
            return None

    # ==========================================
    # 4. CONTROL LOOP & GOAL DISPATCH
    # ==========================================
    def control_loop(self):
        # 1. Check if the escape target frame exists
        target_pose = self.get_escape_pose_in_map()
        if target_pose is None:
            return  # No danger active, do nothing

        # 2. Check if Nav2 action server is reachable
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(
                f"Waiting for action server '{self.action_name}'...", 
                throttle_duration_sec=5.0
            )
            return

        # 3. Prevent goal spamming while awaiting server response
        if self.is_goal_pending:
            return

        # 4. Sending goal to Nav2 Action Server
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.get_logger().info(
            f"ESCAPE ACTIVE! Dispatching Goal -> X: {target_pose.pose.position.x:.2f}, Y: {target_pose.pose.position.y:.2f}"
        )
        self.is_goal_pending = True

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.is_goal_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the escape goal request.")
            return
        self.get_logger().info("Nav2 accepted escape goal! Robot moving to safe point.")

        

# ==========================================
# EXECUTION ENTRYPOINT
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = GoalSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()