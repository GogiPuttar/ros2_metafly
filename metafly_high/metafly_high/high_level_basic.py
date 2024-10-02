#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from metafly_interfaces.msg import Controls  # Import the Controls message

class HighLevelBasic(Node):
    def __init__(self):
        super().__init__('high_level_basic')
        self.get_logger().info('High-level basic node initialized')

        # Subscriber to the Pose data
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'metafly_pose',
            self.pose_callback,
            10
        )

        # Publisher for the cmd_controls topic (Controls messages)
        self.cmd_controls_publisher = self.create_publisher(Controls, 'cmd_controls', 10)

        # Timer to publish at 30 Hz (every 1/30th of a second)
        self.create_timer(1/30.0, self.timer_callback)

        # Variables to store the current pose data and time of last received pose
        self.current_pose = None
        self.last_pose_time = None
        self.max_feedback_latency = 1.0 # seconds

    def pose_callback(self, msg):
        # Store the current pose
        self.current_pose = msg
        self.get_logger().debug(f"Received Pose: {msg.pose}")

    def timer_callback(self):

        # Check if we have received a pose
        if self.current_pose is not None:
            # Get current time
            current_time = self.get_clock().now()

            # Check if more than 1 second has passed since the last received pose
            if self.last_pose_time is not None and (current_time - self.last_pose_time).nanoseconds * 1e-9 > self.max_feedback_latency:
                self.get_logger().warn("No pose received for more than 1 second, stopping publishing.")
                return  # Stop publishing

            # Create a Controls message with zero speed and steering
            controls_msg = Controls()
            controls_msg.speed = 0      # Zero speed
            controls_msg.steering = 0   # Zero steering

            # Publish the zero Controls command
            self.cmd_controls_publisher.publish(controls_msg)
            self.get_logger().debug("Publishing zero Controls command")
        else:
            self.get_logger().warn("No pose received yet, not publishing.")

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelBasic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
