#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker  # Import the Marker message
from metafly_interfaces.msg import Controls  # Import the Controls message
from std_msgs.msg import ColorRGBA  # For defining the marker color

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

        # Publisher for the visualization markers (for RViz)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to publish at 30 Hz (every 1/30th of a second)
        self.create_timer(1/30.0, self.timer_callback)

        # Publish bounding box visualization every 2 seconds
        self.create_timer(1/30.0, self.publish_bounding_box)

        # Variables to store the current pose data and time of last identical pose
        self.current_pose = None
        self.previous_pose = None
        self.pose_identical_time = None
        self.max_identical_duration = 0.5  # seconds

        # Declare constants used in operation
        self.max_feedback_latency = 1.0     # seconds
        self.x_constraints = [-3.0, 1.3]    # [m, m]
        self.y_constraints = [-1.3, 1.7]    # [m, m]
        self.z_constraints = [0.05, 2.0]    # [m, m]
        self.baseline_speed = 18            # [0, 127]
        self.baseline_steering = 0         # [-127, 127]
        self.tracking_state = "untracked"   # "untracked", "out_of_bounds", or "ok"

    def pose_callback(self, msg):

        # Check if the new pose is identical to the previous pose
        if self.previous_pose is not None and self.is_pose_identical(self.previous_pose, msg):
            # If the pose is identical, update the identical pose time if not set
            if self.pose_identical_time is None:
                self.pose_identical_time = self.get_clock().now()
        else:
            # If the pose is different, reset the identical pose time
            self.pose_identical_time = None

        # Store the current pose as previous for the next check
        self.previous_pose = msg
        self.current_pose = msg
        self.get_logger().debug(f"Received Pose: {msg.pose}")

    def timer_callback(self):

        # Check if we have received a pose
        if self.current_pose is not None:
            # Check if the pose has remained identical for more than 1 second
            if self.pose_identical_time is not None:
                current_time = self.get_clock().now()
                if (current_time - self.pose_identical_time).nanoseconds * 1e-9 > self.max_identical_duration:
                    self.get_logger().warn("Pose has been identical for more than 1 second, stopping publishing.")
                    self.tracking_state = "untracked"
                    self.publish_zero_controls()
                    return

            # Check if the bird is within the box
            if self.is_pose_out_of_bounds(self.current_pose.pose):
                self.get_logger().warn("The bird is out of its limits, publishing zero controls.")
                self.tracking_state = "out_of_bounds"
                self.publish_zero_controls()
                return
            
            self.tracking_state = "ok"

            # Publish the controls command
            self.publish_controls(self.baseline_speed, self.baseline_steering)
        else:
            self.get_logger().warn("No pose received yet, not publishing.")

    def is_pose_identical(self, pose1, pose2):
        # Check if the positions of two poses are identical
        return pose1.pose.position.x == pose2.pose.position.x and \
               pose1.pose.position.y == pose2.pose.position.y and \
               pose1.pose.position.z == pose2.pose.position.z and \
               pose1.pose.orientation.x == pose2.pose.orientation.x and \
               pose1.pose.orientation.y == pose2.pose.orientation.y and \
               pose1.pose.orientation.z == pose2.pose.orientation.z and \
               pose1.pose.orientation.w == pose2.pose.orientation.w

    def is_pose_out_of_bounds(self, pose):
        # Check if the pose is outside the defined bounding box
        return pose.position.x < self.x_constraints[0] or pose.position.x > self.x_constraints[1] or \
               pose.position.y < self.y_constraints[0] or pose.position.y > self.y_constraints[1] or \
               pose.position.z < self.z_constraints[0] or pose.position.z > self.z_constraints[1]

    def publish_zero_controls(self):
        controls_msg = Controls()
        controls_msg.speed = 0  # Zero speed
        controls_msg.steering = 0  # Zero steering
        self.cmd_controls_publisher.publish(controls_msg)
        self.get_logger().debug("Publishing zero Controls command")

    def publish_controls(self, speed, steering):
        controls_msg = Controls()
        controls_msg.speed = speed
        controls_msg.steering = steering
        self.cmd_controls_publisher.publish(controls_msg)
        self.get_logger().debug(f"Publishing Controls command: Speed={speed}, Steering={steering}")

    def publish_bounding_box(self):
        # Create a marker for the bounding box
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use lines to represent the bounding box
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width

        # Set the color of the marker
        # Green for within bounds and red for out of bounds
        if self.tracking_state == "untracked":
            marker.color = ColorRGBA(r=1.0, g=0.33, b=0.0, a=0.8)
        elif self.tracking_state == "out_of_bounds":
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        elif self.tracking_state == "ok":
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        else:
            self.get_logger().error("Improper handling of tracking state variable!")
            return

        # Define the corners of the bounding box
        corners = [
            (self.x_constraints[0], self.y_constraints[0], self.z_constraints[0]),
            (self.x_constraints[1], self.y_constraints[0], self.z_constraints[0]),
            (self.x_constraints[1], self.y_constraints[1], self.z_constraints[0]),
            (self.x_constraints[0], self.y_constraints[1], self.z_constraints[0]),
            (self.x_constraints[0], self.y_constraints[0], self.z_constraints[0]),  # Close bottom plane

            (self.x_constraints[0], self.y_constraints[0], self.z_constraints[1]),
            (self.x_constraints[1], self.y_constraints[0], self.z_constraints[1]),
            (self.x_constraints[1], self.y_constraints[1], self.z_constraints[1]),
            (self.x_constraints[0], self.y_constraints[1], self.z_constraints[1]),
            (self.x_constraints[0], self.y_constraints[0], self.z_constraints[1]),  # Close top plane

            (self.x_constraints[0], self.y_constraints[0], self.z_constraints[0]),
            (self.x_constraints[1], self.y_constraints[0], self.z_constraints[0]),
            (self.x_constraints[1], self.y_constraints[0], self.z_constraints[1]),
            (self.x_constraints[1], self.y_constraints[1], self.z_constraints[1]),
            (self.x_constraints[1], self.y_constraints[1], self.z_constraints[0]),
            (self.x_constraints[0], self.y_constraints[1], self.z_constraints[0]),
            (self.x_constraints[0], self.y_constraints[1], self.z_constraints[1]),
        ]

        # Add the points to the marker
        for corner in corners:
            point = Point()
            point.x, point.y, point.z = corner
            marker.points.append(point)

        # Publish the marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelBasic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
