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

class HighLevelReturning(Node):
    def __init__(self):
        super().__init__('high_level_returning')
        self.get_logger().info('High-level returning node initialized')

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
        self.ellipse_marker_publisher = self.create_publisher(Marker, 'ellipse_marker', 10)
        self.tangent_marker_publisher = self.create_publisher(Marker, 'return_tangent_marker', 10)
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
        self.return_ellipse_center = [0.5, -0.3]
        self.return_ellipse_semiaxes = [1.7, 1.5]

        # Parameters for circle fitting
        self.window_size = 50  # Number of points to consider for circle fitting, 50 by default
        self.max_sensible_radius = 4.0  # Maximum allowable radius for center correction
        self.current_radius = 0.0
        self.feedforward_radius = 1.0
        self.target_radius = self.feedforward_radius
        self.max_radius = self.feedforward_radius + 0.5
        self.min_radius = self.feedforward_radius - 0.5

        self.tracking_state = "untracked"   # "untracked", "out_of_bounds", "return", or "ok"

        self.target = Point(x=0.5, y=-0.3, z=0.0)
        self.center = Point(x=0.0, y=0.0, z=0.0)

        self.control_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.control_vector_max = 1.0
        self.control_vector_min = -1.0
        self.length = 0.0

        # Counter for lower frequency calculations
        self.counter = 0
        self.calculation_interval = 1  # Run calculations every nth timer call

        self.feedforward_roll = 0.6
        self.target_roll = self.feedforward_roll
        self.max_roll = self.feedforward_roll + 0.0 # These are not the droids you're looking for
        self.min_roll = self.feedforward_roll - 0.0
        self.prev_error_roll = 0.0
        self.error_roll_dot = 0.0

        self.default_z = 1.2
        self.target_z = self.default_z
        self.max_z = self.default_z + 0.0
        self.min_z = self.default_z - 0.0
        self.current_z = 0.0

        self.min_steering = -127
        self.max_steering = 127
        self.min_speed = 0
        self.max_speed = 127
        # self.max_speed = 100
        # self.max_speed = 90
        # self.max_speed = 80
        self.feedforward_speed = 80

        self.yaw_threshold = np.pi / 4.0

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
            
            if self.is_pose_out_of_ellipse(self.current_pose.pose):
                self.get_logger().debug("The bird will return.")
                self.tracking_state = "return"
                # self.publish_controls(0, 0)
                # return
            else:
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

                    if distance_to_center > self.max_sensible_radius:
                        scale_factor = self.max_sensible_radius / distance_to_center
                        self.center.x = current_x + (self.center.x - current_x) * scale_factor
                        self.center.y = current_y + (self.center.y - current_y) * scale_factor
                        self.current_radius = self.max_sensible_radius

                    # Calculate control vector
                    dx = self.center.x - current_x
                    dy = self.center.y - current_y

                    target_dx = self.target.x - self.center.x
                    target_dy = self.target.y - self.center.y

                    self.length = max(self.control_vector_min, 
                                min(
                                    dx * target_dx + dy * target_dy, 
                                    self.control_vector_max
                                    )
                                )
                    direction_norm = np.sqrt(dx**2 + dy**2) + 1e-9
                    unit_dx = dx / direction_norm
                    unit_dy = dy / direction_norm

                    self.control_vector.x = self.length * unit_dx
                    self.control_vector.y = self.length * unit_dy

            self.publish_trajectory_marker()
            self.publish_center_marker()
            self.publish_target_marker()
            self.publish_control_vector_marker()
            self.publish_ellipse_marker()

            if self.tracking_state == "ok":

                # self.roll_control()
                # self.radius_control()
                self.radius_and_height_control(self.target_radius, self.target_z)
                # self.drift_control_fullstack()

            elif self.tracking_state == "return":

                self.publish_return_tangent_marker()
                # self.radius_and_height_control(self.target_radius, self.target_z)
                self.return_maneuver()

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
    
    def is_pose_out_of_ellipse(self, pose):
        """
        Checks if the given pose is outside the defined ellipse.
        
        The ellipse is defined by:
        - `self.return_ellipse_center`: [x_center, y_center]
        - `self.return_ellipse_semiaxes`: [semi_major_axis, semi_minor_axis]
        
        Parameters:
        - pose: The pose to check (geometry_msgs.msg.Pose)

        Returns:
        - True if the pose is outside the ellipse, False otherwise.
        """
        # Get the x and y coordinates of the current pose
        x = pose.position.x
        y = pose.position.y

        # Get the center and semiaxes of the ellipse
        x_center, y_center = self.return_ellipse_center
        semi_x_axis, semi_y_axis = self.return_ellipse_semiaxes

        # Compute the normalized position with respect to the center of the ellipse
        normalized_x = (x - x_center) / semi_x_axis
        normalized_y = (y - y_center) / semi_y_axis

        # Check if the point is inside the ellipse
        # The equation of the ellipse is (x/a)^2 + (y/b)^2 <= 1
        if (normalized_x ** 2 + normalized_y ** 2) <= 1:
            return False  # Inside the ellipse
        else:
            return True   # Outside the ellipse

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


    def return_maneuver(self):
        """
        Implements a yaw controller over a roll controller to align with the tangent direction.
        This function should be triggered when the current yaw is within ±pi/4 of the tangent direction.
        """
        current_roll, current_pitch, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        # Vector from current position to the center of the circle
        dx = self.target.x - current_x
        dy = self.target.y - current_y
        distance = np.sqrt(dx**2 + dy**2)

        # Check if the current position is outside the circle (distance > radius)
        if distance > self.feedforward_radius:
            # Calculate the angle from the current position to the center
            angle_to_center = np.arctan2(dy, dx)

            # Angle to the tangent points relative to the angle to the center
            theta = np.arcsin(self.feedforward_radius / distance)

            # Calculate the two tangent directions
            tangent_angle1 = angle_to_center + theta
            tangent_angle2 = angle_to_center - theta

            # Choose the tangent that points in the general direction of motion
            # Here we choose the first tangent for simplicity
            # tangent_dx = np.cos(tangent_angle1)
            # tangent_dy = np.sin(tangent_angle1)

            # tangent_end_x = current_x + tangent_dx * self.feedforward_radius
            # tangent_end_y = current_y + tangent_dy * self.feedforward_radius

            # Determine the color based on whether the return maneuver is triggered
            current_yaw = tf_transformations.euler_from_quaternion([
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ])[2]

            yaw_difference = (current_yaw - tangent_angle1 + np.pi) % (2 * np.pi) - np.pi # Normalize to [-pi, pi]

            # Check if the current yaw is within the specified range
            if abs(yaw_difference) <= self.yaw_threshold:

                error_yaw = yaw_difference

                yaw_kp = 0.3
                # self.target_roll = self.feedforward_roll + yaw_kp * error_yaw # maybe remove feedforward roll?
                self.target_roll = yaw_kp * error_yaw # maybe remove feedforward roll?

                error_roll = self.target_roll - current_roll
                alpha = 0.5
                self.error_roll_dot = alpha * (error_roll - self.prev_error_roll) * 0.01 + (1.0 - alpha) * self.error_roll_dot
                self.prev_error_roll = error_roll

                roll_kp = 81.0
                # roll_kd = 50000.0
                roll_kd = 0
                steering_command = int(min(max(self.min_steering, roll_kp * error_roll + roll_kd * self.error_roll_dot), self.max_steering))

                # speed_command = self.feedforward_speed  # Maintain speed during return
                error_z = self.target_z - self.current_z
                z_kp  = 50.0

                speed_command = int(min(max(self.min_speed, self.feedforward_speed + z_kp * error_z), self.max_speed))

                self.get_logger().info(f"Returning: Yaw Diff: {yaw_difference:.2f}, {yaw_kp * error_yaw:.2f}, Speed: {speed_command:.2f}, Steering: {steering_command}")
                self.publish_controls(speed_command, steering_command)
            else:
                # self.get_logger().info(f"Return maneuver not triggered: Yaw difference out of range. {yaw_difference}")
                self.radius_and_height_control(0.1, self.target_z)

        else: 
            self.get_logger().warn(f"Ellipse and target circle are intersecting!")

    def drift_control_fullstack(self):

        current_roll, current_pitch, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        # control_vector_len = (self.control_vector.x**2 + self.control_vector.y**2)**0.5

        drift_kp = 1.0
        # Sharper turn
        if self.length >= 0:
            self.target_radius = self.feedforward_radius + drift_kp * self.length / (self.control_vector_max) * (self.max_radius - self.feedforward_radius)
        # Smoother turn
        elif self.length < 0:
            self.target_radius = self.feedforward_radius + drift_kp * self.length / (self.control_vector_min) * (self.min_radius - self.feedforward_radius)

        error_radius = self.current_radius - self.target_radius # positive radius error should imply positive roll change

        radius_kp = 0.3
        self.target_roll = self.feedforward_roll + radius_kp * error_radius

        error_roll = self.target_roll - current_roll

        roll_kp = 81.0
        steering_command = int(min(max(self.min_steering, roll_kp * error_roll), self.max_steering))

        error_z = self.target_z - self.current_z
        z_kp  = 50.0

        speed_command = int(min(max(self.min_speed, self.feedforward_speed + z_kp * error_z), self.max_speed))

        self.get_logger().info(f"spe:{speed_command}, ste: {steering_command}, roll:{self.target_roll:.2f}, {current_roll: .2f}, rad:{self.target_radius:.2f}")

        self.publish_controls(speed_command, steering_command)

    def radius_and_height_control(self, radius_desired, height_desired):

        current_roll, current_pitch, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        error_radius = self.current_radius - radius_desired # positive radius error should imply positive roll change

        radius_kp = 0.5 # 0.3
        self.target_roll = self.feedforward_roll + radius_kp * error_radius

        error_roll = self.target_roll - current_roll
        alpha = 0.5
        self.error_roll_dot = alpha * (error_roll - self.prev_error_roll) * 0.01 + (1.0 - alpha) * self.error_roll_dot
        self.prev_error_roll = error_roll

        # roll_kp = 81.0
        roll_kp = 100.0
        # roll_kd = 50000.0
        roll_kd = 0
        steering_command = int(min(max(self.min_steering, roll_kp * error_roll + roll_kd * self.error_roll_dot), self.max_steering))

        error_z = height_desired - self.current_z
        z_kp  = 50.0

        speed_command = int(min(max(self.min_speed, self.feedforward_speed + z_kp * error_z), self.max_speed))

        self.get_logger().info(f"spe:{speed_command}, ste: {steering_command, int(roll_kd * self.error_roll_dot)}, roll:{self.target_roll:.2f}, {current_roll: .2f}, rad:{self.current_radius:.2f}")

        self.publish_controls(speed_command, steering_command)

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

        self.get_logger().info(f"spe:{speed_command}, ste: {steering_command}, roll:{self.target_roll:.2f}, {current_roll: .2f}, rad:{self.target_radius:.2f}")

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
        if self.length >= 0:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange for positive length
        else:
            marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0)  # Purple for negative length

        marker.points.append(Point(x=self.center.x, y=self.center.y, z=0.0))  # Start point
        marker.points.append(Point(x=end_x, y=end_y, z=0.0))  # End point

        self.control_vector_marker_publisher.publish(marker)

    def publish_ellipse_marker(self):
        """
        Publishes an RViz marker representing the ellipse defined by:
        - `self.return_ellipse_center`
        - `self.return_ellipse_semiaxes`
        """
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ellipse"
        marker.id = 5  # Unique ID for the ellipse marker
        marker.type = Marker.CYLINDER  # Use a cylinder to represent the ellipse
        marker.action = Marker.ADD
        marker.pose.position.x = self.return_ellipse_center[0]
        marker.pose.position.y = self.return_ellipse_center[1]
        marker.pose.position.z = 0.0  # Flat on the XY plane

        # Orientation to make it flat on the XY plane (no rotation needed)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale to match the ellipse's semiaxes and thickness
        marker.scale.x = 2 * self.return_ellipse_semiaxes[0]  # Full length along X
        marker.scale.y = 2 * self.return_ellipse_semiaxes[1]  # Full length along Y
        marker.scale.z = 0.01  # Small thickness for a flat shape

        # Set the color of the ellipse
        marker.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.2)  # Light blue color

        # Publish the marker
        self.ellipse_marker_publisher.publish(marker)

    def publish_return_tangent_marker(self):
        """
        Publishes a marker representing the tangent line from the current pose to the target circle.
        The color of the tangent will be green when the return maneuver is triggered and red otherwise.
        """
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        # Vector from current position to the center of the circle
        dx = self.target.x - current_x
        dy = self.target.y - current_y
        distance = np.sqrt(dx**2 + dy**2)

        # Check if the current position is outside the circle (distance > radius)
        if distance > self.feedforward_radius:
            # Calculate the angle from the current position to the center
            angle_to_center = np.arctan2(dy, dx)

            # Angle to the tangent points relative to the angle to the center
            theta = np.arcsin(self.feedforward_radius / distance)

            # Calculate the two tangent directions
            tangent_angle1 = angle_to_center + theta
            tangent_angle2 = angle_to_center - theta

            # Choose the tangent that points in the general direction of motion
            # Here we choose the first tangent for simplicity
            tangent_dx = np.cos(tangent_angle1)
            tangent_dy = np.sin(tangent_angle1)

            tangent_end_x = current_x + tangent_dx * self.feedforward_radius
            tangent_end_y = current_y + tangent_dy * self.feedforward_radius

            # Determine the color based on whether the return maneuver is triggered
            current_yaw = tf_transformations.euler_from_quaternion([
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ])[2]

            yaw_difference = abs((current_yaw - tangent_angle1 + np.pi) % (2 * np.pi) - np.pi)

            if yaw_difference <= self.yaw_threshold:  # Check if current yaw is within ±pi/4 of tangent direction
                tangent_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green when return is triggered
            else:
                tangent_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red otherwise

            # Create the marker
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "return_tangent"
            marker.id = 6
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            marker.color = tangent_color

            # Add the start and end points of the tangent
            marker.points.append(Point(x=current_x, y=current_y, z=0.0))  # Start at current pose
            marker.points.append(Point(x=tangent_end_x, y=tangent_end_y, z=0.0))  # End at tangent point

            self.tangent_marker_publisher.publish(marker)


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
        elif self.tracking_state == "return":
            marker.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.8)
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
    node = HighLevelReturning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
