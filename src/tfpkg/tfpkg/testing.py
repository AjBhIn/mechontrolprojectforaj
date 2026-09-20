#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class CostmapTesterNode(Node):
    def __init__(self):
        super().__init__('costmap_tester_node')

        # Map metadata storage
        self.map_data = None
        self.map_res = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # ROS 2 Costmap Topic Subscription
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/our_bot/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        # 1 Hz Timer to inspect cell costs continuously
        self.timer = self.create_timer(1.0, self.test_cost_function_loop)
        self.get_logger().info("Costmap Tester Node started. Waiting for map data...")

    def costmap_callback(self, msg: OccupancyGrid):
        """Receives map message and updates grid dimensions and 1D data array."""
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.map_res = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def get_cell_cost(self, x: float, y: float) -> int:
        """
        Converts real-world meters (x, y) into a 1D array index cost.
        Returns 255 if unmapped (-1) or out of bounds.
        """
        if self.map_data is None:
            return 255  # Map hasn't arrived yet

        # 1. Meters -> 2D Grid Indices (Columns and Rows)
        raw_col = int((x - self.map_origin_x) / self.map_res)
        raw_row = int((y - self.map_origin_y) / self.map_res)

        # 2. Clamping (Squeezing indices to stay inside valid map bounds)
        col = max(0, min(raw_col, self.map_width - 1))
        row = max(0, min(raw_row, self.map_height - 1))

        # 3. Flatten 2D (row, col) into 1D array index
        flat_index = row * self.map_width + col

        # 4. Extract cost value
        cost = self.map_data[flat_index]

        # In ROS, -1 means unknown/unmapped space -> treat as lethal (255)
        return 255 if cost == -1 else int(cost)

    def test_cost_function_loop(self):
        """Demonstrates get_cell_cost by sampling test coordinates every second."""
        if self.map_data is None:
            self.get_logger().info("Waiting for OccupancyGrid topic...")
            return

        # Sample coordinates to query in meters (e.g., origin, nearby room points)
        test_points = [
            (0.0, 0.0),
            (1.5, 0.5),
            (-2.0, -1.0)
        ]

        self.get_logger().info("=" * 45)
        for x, y in test_points:
            cost = self.get_cell_cost(x, y)

            # Interpret cost
            if cost == 0:
                status = "FREE SPACE (Safe)"
            elif cost < 80:
                status = "INFLATION ZONE (Near Wall)"
            elif cost < 255:
                status = "LETHAL OBSTACLE (Wall)"
            else:
                status = "UNKNOWN / UNMAPPED"

            self.get_logger().info(f"Point ({x:+.2f}m, {y:+.2f}m) ──► Cost: {cost:3d} | Status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = CostmapTesterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()