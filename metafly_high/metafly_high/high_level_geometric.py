#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker
from metafly_interfaces.msg import Controls
from std_msgs.msg import ColorRGBA
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
        self.target_marker_publisher = self.create_publisher(Marker, 'target_marker', 10)
        self.box_marker_publisher = self.create_publisher(Marker, 'bounding_box_marker', 10)

        # Timer to publish at 100 Hz
        self.create_timer(1/100.0, self.timer_callback)

        # Publish bounding box visualization every 1/100 seconds
        self.create_timer(1/100.0, self.publish_bounding_box)

        # Variables to store the current and historical pose data
        self.current_pose = None
        self.previous_pose = None
        self.pose_history = []
        self.pose_identical_time = None
        self.max_identical_duration = 1.5  # seconds

        # Define bounding box constraints
        self.x_constraints = [-2.7, 3.7]  # [m, m]
        self.y_constraints = [-2.5, 1.7]  # [m, m]
        self.z_constraints = [0.15, 2.7]   # [m, m]

        # Parameters for circle fitting
        self.window_size = 50  # Number of points to consider for circle fitting
        self.max_radius = 4.0  # Maximum allowable radius for center correction

        self.tracking_state = "untracked"   # "untracked", "out_of_bounds", or "ok"

        self.target = Point(x=0.0, y=0.0, z=0.0)

        self.center = Point(x=0.0, y=0.0, z=0.0)

        self.control_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.control_vector_max = 1.0
        self.control_vector_min = -1.0

        # Counter for lower frequency calculations
        self.counter = 0
        self.calculation_interval = 1  # Run calculations every nth timer call

        self.feedforward_roll = 0.5
        self.target_roll = self.feedforward_roll
        self.max_roll = self.feedforward_roll + 0.0
        self.min_roll = self.feedforward_roll - 0.0
        self.current_radius = 0.0
        self.target_radius = 1.0

        self.default_z = 1.2
        self.target_z = self.default_z
        self.max_z = self.default_z + 0.0
        self.min_z = self.default_z - 0.0
        self.current_z = 0.0

        self.min_steering = -127
        self.max_steering = 127
        self.min_speed = 0
        # self.max_speed = 127
        # self.max_speed = 100
        # self.max_speed = 90
        self.max_speed = 80

    def pose_callback(self, msg):
        # Check if the new pose is identical to the previous pose
        if self.previous_pose is not None and self.is_pose_identical(self.previous_pose, msg):
            if self.pose_identical_time is None:
                self.pose_identical_time = self.get_clock().now()
        else:
            self.pose_identical_time = None

        self.previous_pose = msg
        self.current_pose = msg
        self.pose_history.append(msg)

        # Keep only the latest window_size poses
        if len(self.pose_history) > self.window_size:
            self.pose_history.pop(0)

        self.get_logger().debug(f"Received Pose: {msg.pose}")

    def timer_callback(self):

        self.counter += 1

        if self.current_pose is not None:
            if self.pose_identical_time is not None:
                current_time = self.get_clock().now()
                if (current_time - self.pose_identical_time).nanoseconds * 1e-9 > self.max_identical_duration:
                    self.get_logger().debug(f"Pose has been identical for more than {self.max_identical_duration} second(s), stopping updates.")
                    self.tracking_state = "untracked"
                    self.publish_controls(0, 0)

                    return

            if self.is_pose_out_of_bounds(self.current_pose.pose):
                self.get_logger().debug("The bird is out of bounds.")
                self.tracking_state = "out_of_bounds"
                self.publish_controls(0, 0)
                return
            
            self.tracking_state = "ok"

            # Perform center and control vector calculations at a lower frequency
            if self.counter % self.calculation_interval == 0:

                # Calculate and publish visualization markers
                if len(self.pose_history) >= 3:
                    x = np.array([pose.pose.position.x for pose in self.pose_history])
                    y = np.array([pose.pose.position.y for pose in self.pose_history])

                    # Calculate current center
                    self.center.x, self.center.y, _ = self.fit_circle(x, y)
                    current_x = self.current_pose.pose.position.x
                    current_y = self.current_pose.pose.position.y
                    self.current_z = self.current_pose.pose.position.z

                    distance_to_center = np.sqrt((self.center.x - current_x)**2 + (self.center.y - current_y)**2)
                    self.current_radius = distance_to_center

                    if distance_to_center > self.max_radius:
                        scale_factor = self.max_radius / distance_to_center
                        self.center.x = current_x + (self.center.x - current_x) * scale_factor
                        self.center.y = current_y + (self.center.y - current_y) * scale_factor
                        self.current_radius = self.max_radius

                    # Calculate control vector
                    dx = self.center.x - current_x
                    dy = self.center.y - current_y

                    target_dx = self.target.x - self.center.x
                    target_dy = self.target.y - self.center.y

                    length = max(self.control_vector_min, 
                                min(
                                    dx * target_dx + dy * target_dy, 
                                    self.control_vector_max
                                    )
                                )
                    direction_norm = np.sqrt(dx**2 + dy**2) + 1e-9
                    unit_dx = dx / direction_norm
                    unit_dy = dy / direction_norm

                    self.control_vector.x = length * unit_dx
                    self.control_vector.y = length * unit_dy

                    self.publish_trajectory_marker()
                    self.publish_center_marker()
                    self.publish_target_marker()
                    self.publish_control_vector_marker()

            # self.roll_control()
            self.radius_control()

    def is_pose_identical(self, pose1, pose2):
        return pose1.pose.position.x == pose2.pose.position.x and \
               pose1.pose.position.y == pose2.pose.position.y and \
               pose1.pose.position.z == pose2.pose.position.z and \
               pose1.pose.orientation.x == pose2.pose.orientation.x and \
               pose1.pose.orientation.y == pose2.pose.orientation.y and \
               pose1.pose.orientation.z == pose2.pose.orientation.z and \
               pose1.pose.orientation.w == pose2.pose.orientation.w

    def is_pose_out_of_bounds(self, pose):
        return pose.position.x < self.x_constraints[0] or pose.position.x > self.x_constraints[1] or \
               pose.position.y < self.y_constraints[0] or pose.position.y > self.y_constraints[1] or \
               pose.position.z < self.z_constraints[0] or pose.position.z > self.z_constraints[1]

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

    def radius_control(self):

        current_roll, current_pitch, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        error_radius = self.current_radius - self.target_radius # positive radius error should imply positive roll change

        radius_kp = 0.3
        self.target_roll = self.feedforward_roll + radius_kp * error_radius

        error_roll = self.target_roll - current_roll

        # Limit the output to the range [-127, 127]
        speed_command = self.max_speed

        roll_kp = 81.0
        steering_command = int(min(max(self.min_steering, roll_kp * error_roll), self.max_steering))

        # self.get_logger().info(f"heyy: {error_roll}")
        self.get_logger().info(f"heyy: {self.target_roll, steering_command}")

        self.publish_controls(speed_command, steering_command)

    def roll_control(self):

        current_roll, current_pitch, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        control_vector_len = (self.control_vector.x**2 + self.control_vector.y**2)**0.5

        if control_vector_len >= 0:
            self.target_roll = self.feedforward_roll + control_vector_len / (self.control_vector_max) * (self.max_roll - self.feedforward_roll)
        elif control_vector_len < 0:
            self.target_roll = self.feedforward_roll + control_vector_len / (self.control_vector_min) * (self.min_roll - self.feedforward_roll)

        error_roll = self.target_roll - current_roll

        # Limit the output to the range [-127, 127]
        speed_command = self.max_speed

        roll_kp = 81
        steering_command = int(min(max(self.min_steering, roll_kp * error_roll), self.max_steering))

        # self.get_logger().info(f"heyy: {error_roll}")
        self.get_logger().info(f"heyy: {steering_command}")

        self.publish_controls(speed_command, steering_command)

    def publish_trajectory_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # Line width
        marker.color = ColorRGBA(r=0.4, g=0.4, b=0.9, a=0.9)  # Blue color

        for pose in self.pose_history:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z
            marker.points.append(point)

        self.trajectory_marker_publisher.publish(marker)

    def publish_center_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "center"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.center.x
        marker.pose.position.y = self.center.y
        marker.pose.position.z = 0.0  # Projected on the XY plane
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)  # Size of the sphere
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)  # Green color

        self.center_marker_publisher.publish(marker)

    def publish_target_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.target.x
        marker.pose.position.y = self.target.y
        marker.pose.position.z = 0.0  # Projected on the XY plane
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)  # Size of the sphere
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)  # Red color

        self.target_marker_publisher.publish(marker)

    def publish_control_vector_marker(self):
        
        end_x = self.center.x + self.control_vector.x
        end_y = self.center.y + self.control_vector.y

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "control_vector"
        marker.id = 4
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1   # Arrowhead diameter
        marker.scale.z = 0.1   # Arrowhead length

        # Set arrow color based on length
        if (self.control_vector.x**2 + self.control_vector.y**2)**0.5 > 0:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange for positive length
        else:
            marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0)  # Purple for negative length

        marker.points.append(Point(x=self.center.x, y=self.center.y, z=0.0))  # Start point
        marker.points.append(Point(x=end_x, y=end_y, z=0.0))  # End point

        self.control_vector_marker_publisher.publish(marker)

    def publish_controls(self, speed : int = 0, steering : int = 0):
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
        self.box_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelGeometric()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
