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
from enum import Enum, auto


class State(Enum):
    """
    Current state of the system.
    Determines what the main timer function should be doing on each iteration.
    """
    CIRCLING = auto()        # Robot continues circling using the default control strategy
    UNTRACKED = auto()       # Robot is out of motion capture range
    OUT_OF_BOUNDS = auto()   # Robot is tracked but outside the bounding box
    RETURNING = auto()       # Robot performs a return maneuver toward the ellipse


class HighLevelDrift(Node):
    """
    High-level control node for a bird robot navigating within a defined workspace.
    Handles trajectory management, state transitions, and visualization markers for RViz.
    """

    def __init__(self):
        super().__init__('high_level_drift')
        self.get_logger().info('High-level drift node initialized')

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
        self.target_marker_publisher = self.create_publisher(Marker, 'target_marker', 10)
        self.ellipse_marker_publisher = self.create_publisher(Marker, 'ellipse_marker', 10)
        self.tangent_marker_publisher = self.create_publisher(Marker, 'return_tangent_marker', 10)
        self.box_marker_publisher = self.create_publisher(Marker, 'bounding_box_marker', 10)

        # Timer to publish at 100 Hz
        self.create_timer(1 / 100.0, self.timer_callback)

        # Publish bounding box visualization every 1/100 seconds
        self.create_timer(1 / 100.0, self.publish_bounding_box)

        # Variables to store the current and historical pose data
        self.current_pose = None
        self.previous_pose = None
        self.pose_history = []
        self.pose_identical_time = None
        self.max_identical_duration = 1.5  # seconds

        # Marker parameters
        self.target = Point(x=0.5, y=-0.3, z=0.0)
        self.center = Point(x=0.0, y=0.0, z=0.0)
        self.tangent_direction = Vector3(x=0.0, y=0.0, z=0.0)
        self.tangent_length = 1.0

        # Define bounding box constraints
        self.bounding_box = {
            "x": [-2.7, 3.7],  # [m, m]
            "y": [-2.5, 1.7],  # [m, m]
            "z": [0.15, 2.7],  # [m, m]
        }

        # Define ellipse constraints
        self.return_ellipse = {
            "center": [self.target.x, self.target.y],
            "semiaxes": [1.7, 1.5]
        }

        # Parameters for circle fitting
        self.window_size = 50  # Number of points to consider for circle fitting
        self.max_sensible_radius = 4.0  # Maximum allowable radius for center correction
        self.current_radius = 0.0

        # Tracking state
        self.tracking_state = State.UNTRACKED  # Initial state

        # Parameters for lower frequency updates
        self.counter = 0
        self.calculation_interval = 1  # Run calculations every nth timer call

        # Radius control parameters
        self.feedforward_radius = 1.0
        self.target_radius = self.feedforward_radius

        # Roll control parameters
        self.feedforward_roll = 0.6
        self.target_roll = self.feedforward_roll
        self.prev_error_roll = 0.0
        self.error_roll_dot = 0.0

        # Height control parameters
        self.default_z = 1.2
        self.target_z = self.default_z
        self.current_z = 0.0

        # Return Maneuver parameters
        self.yaw_threshold = np.pi / 4.0
        self.sharp_turn_radius = 0.1

        # Limits
        self.steering = {"max": 127, "min": -127}
        self.speed = {"max": 127, "min": 0}
        self.feedforward_speed = 80

        # Gains
        self.kp_roll = 100.0  # Converts roll (rad) to steering command (-127 to 127)
        self.kd_roll = 0.0    # Converts roll speed (rad/s) to steering command (default: 50,000)
        self.kp_radius = 0.5  # Converts radius (m) to roll (rad)
        self.kp_yaw = 0.3     # Converts yaw (rad) to roll (rad)
        self.kp_z = 50.0      # Converts height (m) to speed command (0 to 127)

    def pose_callback(self, msg):
        """
        Callback function for the PoseStamped subscriber.

        Parameters:
        - msg (PoseStamped): Current pose of the bird robot.
        """
        if self.previous_pose is not None and self.is_pose_identical(self.previous_pose, msg):
            if self.pose_identical_time is None:
                self.pose_identical_time = self.get_clock().now()
        else:
            self.pose_identical_time = None

        self.previous_pose = msg
        self.current_pose = msg
        self.pose_history.append(msg)

        # Maintain a fixed-length pose history
        if len(self.pose_history) > self.window_size:
            self.pose_history.pop(0)

        self.get_logger().debug(f"Received Pose: {msg.pose}")

    def timer_callback(self):
        """
        Main timer callback executed at 100 Hz.
        Manages state transitions and publishes controls/markers based on the current state.
        """
        self.counter += 1

        if self.current_pose is not None:
            # Handle untracked state
            if self.pose_identical_time is not None:
                current_time = self.get_clock().now()
                if (current_time - self.pose_identical_time).nanoseconds * 1e-9 > self.max_identical_duration:
                    self.get_logger().debug(f"Pose untracked for {self.max_identical_duration} seconds. Stopping updates.")
                    self.tracking_state = State.UNTRACKED
                    self.publish_controls(0, 0)
                    return

            # Check bounds and transition states
            if self.is_pose_out_of_bounds(self.current_pose.pose):
                self.get_logger().debug("The bird is out of bounds.")
                self.tracking_state = State.OUT_OF_BOUNDS
                self.publish_controls(0, 0)
                return

            if self.is_pose_out_of_ellipse(self.current_pose.pose):
                self.tracking_state = State.RETURNING
                self.get_logger().debug("Bird entering return state.")
            else:
                self.tracking_state = State.CIRCLING

            # Perform marker and control updates
            if self.counter % self.calculation_interval == 0:
                self.update_center_and_radius()

            self.publish_markers()

            if self.tracking_state == State.CIRCLING:
                self.radius_and_height_control(self.target_radius, self.target_z)
            elif self.tracking_state == State.RETURNING:
                self.publish_return_tangent_marker()
                self.return_maneuver()

    def is_pose_identical(self, pose1, pose2):
        """
        Compares two poses to determine if they are identical.

        Parameters:
        - pose1 (PoseStamped): First pose.
        - pose2 (PoseStamped): Second pose.

        Returns:
        - bool: True if identical, False otherwise.
        """
        return pose1.pose.position.x == pose2.pose.position.x and \
               pose1.pose.position.y == pose2.pose.position.y and \
               pose1.pose.position.z == pose2.pose.position.z and \
               pose1.pose.orientation.x == pose2.pose.orientation.x and \
               pose1.pose.orientation.y == pose2.pose.orientation.y and \
               pose1.pose.orientation.z == pose2.pose.orientation.z and \
               pose1.pose.orientation.w == pose2.pose.orientation.w

    def is_pose_out_of_bounds(self, pose):
        """
        Determines if the given pose is out of the defined bounding box.

        Parameters:
        - pose (Pose): The pose to check.

        Returns:
        - bool: True if the pose is out of bounds, False otherwise.
        """
        return pose.position.x < self.bounding_box["x"][0] or pose.position.x > self.bounding_box["x"][1] or \
               pose.position.y < self.bounding_box["y"][0] or pose.position.y > self.bounding_box["y"][1] or \
               pose.position.z < self.bounding_box["z"][0] or pose.position.z > self.bounding_box["z"][1]

    def is_pose_out_of_ellipse(self, pose):
        """
        Checks if the given pose is outside the defined ellipse.

        Parameters:
        - pose (Pose): The pose to check.

        Returns:
        - bool: True if the pose is outside the ellipse, False otherwise.
        """
        x = pose.position.x
        y = pose.position.y
        x_center, y_center = self.return_ellipse["center"]
        semi_x_axis, semi_y_axis = self.return_ellipse["semiaxes"]
        normalized_x = (x - x_center) / semi_x_axis
        normalized_y = (y - y_center) / semi_y_axis
        return (normalized_x ** 2 + normalized_y ** 2) > 1
    
    def update_center_and_radius(self):
        """
        Updates the circle center and radius based on the pose history.
        This uses the last 'window_size' points to fit a circle and calculates the current radius.

        If the calculated radius exceeds the 'max_sensible_radius', it is constrained, and the center is adjusted.

        Updates:
        - self.center.x
        - self.center.y
        - self.current_radius
        """
        if len(self.pose_history) >= 3:
            # Extract x and y coordinates from pose history
            x = np.array([pose.pose.position.x for pose in self.pose_history])
            y = np.array([pose.pose.position.y for pose in self.pose_history])

            # Fit a circle to the points
            self.center.x, self.center.y, _ = self.fit_circle(x, y)
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y

            # Calculate the radius as the distance from the current pose to the center
            distance_to_center = np.sqrt((self.center.x - current_x) ** 2 + (self.center.y - current_y) ** 2)
            self.current_radius = distance_to_center

            # Constrain the radius if it exceeds the maximum sensible value
            if distance_to_center > self.max_sensible_radius:
                scale_factor = self.max_sensible_radius / distance_to_center
                self.center.x = current_x + (self.center.x - current_x) * scale_factor
                self.center.y = current_y + (self.center.y - current_y) * scale_factor
                self.current_radius = self.max_sensible_radius


    def fit_circle(self, x, y):
        """
        Fits a circle to a set of x, y coordinates using least squares optimization.

        Parameters:
        - x (np.ndarray): x-coordinates of the points.
        - y (np.ndarray): y-coordinates of the points.

        Returns:
        - Tuple[float, float, float]: Center coordinates (x, y) and radius of the fitted circle.
        """
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

    def return_maneuver(self):
        """
        Executes a return maneuver by adjusting yaw and roll control to align with the tangent direction.
        Triggered when the robot is outside the target circle and heading toward the return ellipse.
        """
        current_roll, current_pitch, current_yaw = self.OrientationRPY()
        dx = self.target.x - self.current_pose.pose.position.x
        dy = self.target.y - self.current_pose.pose.position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)

        if distance > self.feedforward_radius:
            angle_to_center = np.arctan2(dy, dx)
            theta = np.arcsin(self.feedforward_radius / distance)
            tangent_angle1 = angle_to_center + theta
            self.tangent_direction.x = np.cos(tangent_angle1)
            self.tangent_direction.y = np.sin(tangent_angle1)
            yaw_difference = (current_yaw - tangent_angle1 + np.pi) % (2 * np.pi) - np.pi

            if abs(yaw_difference) <= self.yaw_threshold:
                speed_command = self.height_P(self.current_pose.pose.position.z, self.target_z, self.feedforward_speed)
                steering_command = self.yaw_P(current_yaw, current_roll, tangent_angle1, feedforward_roll=0.0)
                self.publish_controls(speed_command, steering_command)
            else:
                self.radius_and_height_control(self.sharp_turn_radius, self.target_z)
        else:
            self.get_logger().warn("Ellipse and target circle are intersecting!")

    def radius_and_height_control(self, radius_desired, height_desired):
        """
        Adjusts speed and steering to maintain the target radius and height.

        Parameters:
        - radius_desired (float): Target radius.
        - height_desired (float): Target height.
        """
        current_roll, _, _ = self.OrientationRPY()
        speed_command = self.height_P(self.current_pose.pose.position.z, height_desired, self.feedforward_speed)
        steering_command = self.radius_P(self.current_radius, current_roll, radius_desired, feedforward_roll=self.feedforward_roll)
        self.get_logger().info(f"Speed: {speed_command}, Steering: {steering_command}, Radius: {self.current_radius}")
        self.publish_controls(speed_command, steering_command)

    def height_P(self, current_z, target_z, feedforward_speed):
        """
        Computes the speed command based on the height error using a P-controller.

        Parameters:
        - current_z (float): Current height.
        - target_z (float): Target height.
        - feedforward_speed (int): Feedforward speed bias.

        Returns:
        - int: Constrained speed command.
        """
        error_z = target_z - current_z
        return int(min(max(self.speed["min"], feedforward_speed + self.kp_z * error_z), self.speed["max"]))

    def radius_P(self, current_radius, current_roll, target_radius, feedforward_roll=0):
        """
        Computes the steering command to achieve the desired radius of curvature.

        Parameters:
        - current_radius (float): Current radius.
        - current_roll (float): Current roll.
        - target_radius (float): Target radius.
        - feedforward_roll (float): Feedforward roll bias.

        Returns:
        - int: Steering command.
        """
        error_radius = target_radius - current_radius
        return self.roll_PD(current_roll, target_roll=feedforward_roll + self.kp_radius * -error_radius)

    def yaw_P(self, current_yaw, current_roll, target_yaw, feedforward_roll=0):
        """
        Computes the steering command based on the yaw error using a P-controller.

        Parameters:
        - current_yaw (float): Current yaw.
        - current_roll (float): Current roll.
        - target_yaw (float): Target yaw.
        - feedforward_roll (float): Feedforward roll bias.

        Returns:
        - int: Steering command.
        """
        error_yaw = (current_yaw - target_yaw + np.pi) % (2 * np.pi) - np.pi
        return self.roll_PD(current_roll, target_roll=feedforward_roll + self.kp_yaw * error_yaw)

    def roll_PD(self, current_roll, target_roll, feedforward_steering=0):
        """
        Computes the steering command based on the roll error using a PD-controller.

        Parameters:
        - current_roll (float): Current roll.
        - target_roll (float): Target roll.
        - feedforward_steering (float): Feedforward steering bias.

        Returns:
        - int: Steering command.
        """
        error_roll = target_roll - current_roll
        alpha = 0.5  # Low-pass filter coefficient
        self.error_roll_dot = alpha * (error_roll - self.prev_error_roll) * 0.01 + (1.0 - alpha) * self.error_roll_dot
        self.prev_error_roll = error_roll
        return int(min(max(self.steering["min"], feedforward_steering + self.kp_roll * error_roll + self.kd_roll * self.error_roll_dot), self.steering["max"]))

    def OrientationRPY(self):
        """
        Converts the current pose's quaternion to roll-pitch-yaw (RPY).

        Returns:
        - Tuple[float, float, float]: Roll, pitch, and yaw values.
        """
        return tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

    def publish_markers(self):
        """
        Publishes all visualization markers for RViz.
        """
        self.publish_trajectory_marker()
        self.publish_center_marker()
        self.publish_target_marker()
        self.publish_ellipse_marker()

    def publish_trajectory_marker(self):
        """
        Publishes the trajectory marker for RViz as a line strip.
        """
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
            point = Point(x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z)
            marker.points.append(point)

        self.trajectory_marker_publisher.publish(marker)

    def publish_center_marker(self):
        """
        Publishes the center marker for RViz as a sphere.
        """
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
        """
        Publishes the target marker for RViz as a sphere.
        """
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

    def publish_ellipse_marker(self):
        """
        Publishes an RViz marker representing the ellipse defined by the return constraints.
        """
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ellipse"
        marker.id = 4
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.return_ellipse["center"][0]
        marker.pose.position.y = self.return_ellipse["center"][1]
        marker.pose.position.z = 0.0  # Flat on the XY plane
        marker.pose.orientation.w = 1.0  # Identity quaternion
        marker.scale.x = 2 * self.return_ellipse["semiaxes"][0]
        marker.scale.y = 2 * self.return_ellipse["semiaxes"][1]
        marker.scale.z = 0.01
        marker.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.2)  # Light blue
        self.ellipse_marker_publisher.publish(marker)

    def publish_return_tangent_marker(self):
        """
        Publishes a marker representing the tangent line from the current pose to the target circle.
        The color of the tangent indicates whether the return maneuver is triggered.
        """
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        tangent_end_x = current_x + self.tangent_length * self.tangent_direction.x
        tangent_end_y = current_y + self.tangent_length * self.tangent_direction.y
        current_yaw = self.OrientationRPY()[2]
        yaw_difference = abs((current_yaw - np.arctan2(self.tangent_direction.y, self.tangent_direction.x) + np.pi) % (2 * np.pi) - np.pi)
        tangent_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) if yaw_difference <= self.yaw_threshold else ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "return_tangent"
        marker.id = 5
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color = tangent_color
        marker.points.append(Point(x=current_x, y=current_y, z=0.0))  # Start point
        marker.points.append(Point(x=tangent_end_x, y=tangent_end_y, z=0.0))  # End point
        self.tangent_marker_publisher.publish(marker)

    def publish_controls(self, speed=0, steering=0):
        """
        Publishes control commands (speed and steering) for the robot.

        Parameters:
        - speed (int): Speed command (0 to 127).
        - steering (int): Steering command (-127 to 127).
        """
        controls_msg = Controls()
        controls_msg.speed = speed
        controls_msg.steering = steering
        self.cmd_controls_publisher.publish(controls_msg)
        self.get_logger().debug(f"Publishing Controls: Speed={speed}, Steering={steering}")

    def publish_bounding_box(self):
        """
        Publishes a marker representing the bounding box for RViz.
        The color of the box changes based on the current tracking state.
        """
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        state_colors = {
            State.UNTRACKED: ColorRGBA(r=1.0, g=0.33, b=0.0, a=0.8),
            State.OUT_OF_BOUNDS: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
            State.RETURNING: ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.8),
            State.CIRCLING: ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
        }
        marker.color = state_colors.get(self.tracking_state, ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5))  # Default to white

        corners = [
            (self.bounding_box["x"][0], self.bounding_box["y"][0], self.bounding_box["z"][0]),
            (self.bounding_box["x"][1], self.bounding_box["y"][0], self.bounding_box["z"][0]),
            (self.bounding_box["x"][1], self.bounding_box["y"][1], self.bounding_box["z"][0]),
            (self.bounding_box["x"][0], self.bounding_box["y"][1], self.bounding_box["z"][0]),
            (self.bounding_box["x"][0], self.bounding_box["y"][0], self.bounding_box["z"][0]),  # Close bottom plane

            (self.bounding_box["x"][0], self.bounding_box["y"][0], self.bounding_box["z"][1]),
            (self.bounding_box["x"][1], self.bounding_box["y"][0], self.bounding_box["z"][1]),
            (self.bounding_box["x"][1], self.bounding_box["y"][1], self.bounding_box["z"][1]),
            (self.bounding_box["x"][0], self.bounding_box["y"][1], self.bounding_box["z"][1]),
            (self.bounding_box["x"][0], self.bounding_box["y"][0], self.bounding_box["z"][1]),  # Close top plane

            (self.bounding_box["x"][0], self.bounding_box["y"][0], self.bounding_box["z"][0]),
            (self.bounding_box["x"][1], self.bounding_box["y"][0], self.bounding_box["z"][0]),
            (self.bounding_box["x"][1], self.bounding_box["y"][0], self.bounding_box["z"][1]),
            (self.bounding_box["x"][1], self.bounding_box["y"][1], self.bounding_box["z"][1]),
            (self.bounding_box["x"][1], self.bounding_box["y"][1], self.bounding_box["z"][0]),
            (self.bounding_box["x"][0], self.bounding_box["y"][1], self.bounding_box["z"][0]),
            (self.bounding_box["x"][0], self.bounding_box["y"][1], self.bounding_box["z"][1]),
        ]
        for corner in corners:
            marker.points.append(Point(x=corner[0], y=corner[1], z=corner[2]))
        self.box_marker_publisher.publish(marker)


def main(args=None):
    """
    Main entry point for the high-level drift node.
    Initializes the node and starts spinning.
    """
    rclpy.init(args=args)
    node = HighLevelDrift()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
