#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker  # Import the Marker message
from metafly_interfaces.msg import Controls  # Import the Controls message
from std_msgs.msg import ColorRGBA  # For defining the marker color
import tf_transformations
import numpy as np
from scipy.optimize import least_squares

class HighLevelGeometric(Node):
    def __init__(self):
        super().__init__('high_level_geometric')
        self.get_logger().info('High-level geometric node initialized')

        # Subscriber to the Pose data
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'metafly_pose',
            self.pose_callback,
            10
        )

        # Publisher for the cmd_controls topic (Controls messages)
        self.cmd_controls_publisher = self.create_publisher(Controls, 'cmd_controls', 10)

        # Publishers for RViz visualization markers
        self.trajectory_marker_publisher = self.create_publisher(Marker, 'trajectory_marker', 10)
        self.center_marker_publisher = self.create_publisher(Marker, 'center_marker', 10)
        self.control_vector_marker_publisher = self.create_publisher(Marker, 'control_vector_marker', 10)

        # Timer to publish at 30 Hz (every 1/30th of a second)
        self.create_timer(1/30.0, self.timer_callback)

        # Variables to store the current and historical pose data
        self.current_pose = None
        self.pose_history = []

        # Parameters for circle fitting
        self.window_size = 50  # Number of points to consider for circle fitting
        self.max_radius = 4.0  # Maximum allowable radius for center correction

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_history.append(msg)

        # Keep only the latest window_size poses
        if len(self.pose_history) > self.window_size:
            self.pose_history.pop(0)

    def timer_callback(self):
        if self.current_pose is not None:
            # Calculate the center of rotation if enough data is available
            if len(self.pose_history) >= 3:
                x = np.array([pose.pose.position.x for pose in self.pose_history])
                y = np.array([pose.pose.position.y for pose in self.pose_history])

                # Fit a circle to the trajectory segment
                xc, yc, _ = self.fit_circle(x, y)
                current_x = self.current_pose.pose.position.x
                current_y = self.current_pose.pose.position.y

                # Calculate the distance and limit it to max_radius
                distance_to_center = np.sqrt((xc - current_x)**2 + (yc - current_y)**2)
                if distance_to_center > self.max_radius:
                    scale_factor = self.max_radius / distance_to_center
                    xc = current_x + (xc - current_x) * scale_factor
                    yc = current_y + (yc - current_y) * scale_factor

                # Publish markers for RViz visualization
                self.publish_trajectory_marker()
                self.publish_center_marker(xc, yc)
                self.publish_control_vector_marker(xc, yc, current_x, current_y)

    def fit_circle(self, x, y):
        def calc_radius(xc, yc):
            return np.sqrt((x - xc) ** 2 + (y - yc) ** 2)

        def cost_func(c):
            radii = calc_radius(*c)
            return radii - np.mean(radii)

        center_estimate = np.mean(x), np.mean(y)
        result = least_squares(cost_func, center_estimate)
        xc, yc = result.x
        radii = calc_radius(xc, yc)
        radius = np.mean(radii)
        return xc, yc, radius

    def publish_trajectory_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue color

        # Add points from pose history
        for pose in self.pose_history:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z
            marker.points.append(point)

        self.trajectory_marker_publisher.publish(marker)

    def publish_center_marker(self, xc, yc):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "center"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = xc
        marker.pose.position.y = yc
        marker.pose.position.z = 0.0  # Projected on the XY plane
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)  # Size of the sphere
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green color

        self.center_marker_publisher.publish(marker)

    def publish_control_vector_marker(self, xc, yc, current_x, current_y):
        dx = xc - current_x
        dy = yc - current_y

        target_x, target_y = 0, 0  # Assume a target at the origin for now
        target_dx = target_x - xc
        target_dy = target_y - yc

        length = dx * target_dx + dy * target_dy
        direction_norm = np.sqrt(dx**2 + dy**2)
        unit_dx = dx / direction_norm
        unit_dy = dy / direction_norm
        end_x = xc + length * unit_dx
        end_y = yc + length * unit_dy

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "control_vector"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1   # Arrowhead diameter
        marker.scale.z = 0.1   # Arrowhead length

        # Set arrow color based on length
        if length > 0:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange for positive length
        else:
            marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0)  # Purple for negative length

        # Set arrow start and end points
        marker.points.append(Point(x=xc, y=yc, z=0.0))  # Start point
        marker.points.append(Point(x=end_x, y=end_y, z=0.0))  # End point

        self.control_vector_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelGeometric()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
