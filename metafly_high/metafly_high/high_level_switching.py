#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker  # Import the Marker message
from metafly_interfaces.msg import Controls  # Import the Controls message
from std_msgs.msg import ColorRGBA  # For defining the marker color
import tf_transformations
import numpy as np

class HighLevelSwitching(Node):
    def __init__(self):
        super().__init__('high_level_switching')
        self.get_logger().info('High-level switching node initialized')

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
        self.box_marker_publisher = self.create_publisher(Marker, 'bounding_box_marker', 10)

        # Publisher for the visualization markers (for RViz)
        self.target_marker_publisher = self.create_publisher(Marker, 'target_marker', 10)

        # Timer to publish at 30 Hz (every 1/30th of a second)
        self.create_timer(1/100.0, self.timer_callback)

        # Publish bounding box visualization every 2 seconds
        self.create_timer(1/100.0, self.publish_bounding_box)

        # Variables to store the current pose data and time of last identical pose
        self.current_pose = None
        self.previous_pose = None
        self.pose_identical_time = None
        self.max_identical_duration = 1.5   # seconds

        # Declare constants used in operation
        self.max_feedback_latency = 1.0     # seconds
        # self.x_constraints = [-3.0, 1.3]    # [m, m]
        # self.y_constraints = [-1.1, 1.7]    # [m, m]
        # self.z_constraints = [0.05, 2.0]    # [m, m]
        # self.x_constraints = [-6.0, 3.0]    # [m, m]
        # self.y_constraints = [-2.0, 2.5]    # [m, m]
        # self.z_constraints = [-0.5, 4.0]    # [m, m]

        self.x_constraints = [-3.7, 3.7]  # [m, m]
        self.y_constraints = [-2.5, 1.7]  # [m, m]
        self.z_constraints = [0.2, 2.7]   # [m, m]
        self.baseline_speed = 0             # [0, 127]
        self.baseline_steering = 0          # [-127, 127]
        self.tracking_state = "untracked"   # "untracked", "out_of_bounds", or "ok"

        # Variables for PID control

        # For level
        self.target_z = 1.4  # Target z-level for the bird (fixed)
        self.kp_z = 100.0        # Proportional gain
        self.ki_z = 0.0        # Integral gain
        self.kd_z = 0.0        # Derivative gain
        self.gravcomp_speed = 50

        self.integral_z = 0.0
        self.previous_error_z = None
        self.max_speed = 127
        self.min_speed = 0
        self.desirable_range_z = 0.2  # Desirable range around the target z-level
        self.leveling_state = False

        # For direction
        self.target_yaw = 0.0
        self.kp_yaw = 100.0       # Proportional gain
        # self.kp_yaw = 0.0       # Proportional gain
        self.ki_yaw = 0.0        # Integral gain
        self.kd_yaw = 0.0        # Derivative gain

        self.integral_yaw = 0.0
        self.previous_error_yaw = None
        self.max_steering = 127
        self.min_steering = -127
        self.desirable_range_yaw = 0.2  # Desirable range around the target yaw
        self.directing_state = False

        self.target_roll = 0.0


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
                    self.get_logger().debug("Pose has been identical for more than 1 second, stopping publishing.")
                    self.tracking_state = "untracked"
                    self.publish_zero_controls()
                    return

            # Check if the bird is within the box
            if self.is_pose_out_of_bounds(self.current_pose.pose):
                self.get_logger().debug("The bird is out of its limits, publishing zero controls.")
                self.tracking_state = "out_of_bounds"
                self.publish_zero_controls()
                return
            
            self.tracking_state = "ok"

            # Publish the controls command
            # self.simple_switching()
            self.loop()

            self.publish_target_marker()
        else:
            # self.get_logger().warn("No pose received yet, not publishing.")
            self.get_logger().debug("No pose received yet, not publishing.")

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

    def loop(self):

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.current_pose.pose.position.z
        current_roll, current_pitch, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        x_thresh = 0.5
        yaw_thresh_0 = [-np.pi/2, np.pi/12]
        yaw_thresh_right = [np.pi/12, np.pi/2]

        error_z = self.target_z - current_z
        error_yaw = self.target_yaw - current_yaw
        error_roll = self.target_roll - current_roll

        # Limit the output to the range [-127, 127]
        speed_command = self.max_speed

        if ((current_x > x_thresh) and (yaw_thresh_0[0] < current_yaw < yaw_thresh_0[1]))  :
            steering_command = -127
        elif (yaw_thresh_right[0] < current_yaw < yaw_thresh_right[1]):
            steering_command = 127
        else:
            steering_command = 0

        # Check if the bird is in the desirable range of the target
        if abs(error_z) < self.desirable_range_z:
            self.leveling_state = True
        else:
            self.leveling_state = False
        if abs(error_yaw) < self.desirable_range_yaw:
            self.directing_state_state = True
        else:
            self.directing_state = False

        self.get_logger().info(f"heyy: {error_roll}")

        self.publish_controls(speed_command, steering_command)

    def simple_switching(self):

        current_z = self.current_pose.pose.position.z
        _, _, current_yaw = tf_transformations.euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])

        error_z = self.target_z - current_z
        error_yaw = self.target_yaw - current_yaw

        # Limit the output to the range [-127, 127]
        speed_command = self.max_speed

        # (z_1, c_1), (z_2, c_2), (z_3, c_3)
        # set_points = [(0.75, -50),
        #               (1.4,-36),
        #               (2, -28)]
        set_points = [(0.75, -60),
                      (1.4,-36),
                      (2, -28)]
        
        if error_z > 0:
            steering_gain = (set_points[2][1] - set_points[1][1]) / (set_points[2][0] - set_points[1][0])
        else:
            steering_gain = (set_points[1][1] - set_points[0][1]) / (set_points[1][0] - set_points[0][0])

        steering_command = int(set_points[1][1] + steering_gain * error_z)
        # steering_command = 0
        # steering_command = -127
        steering_command = -28

        # Check if the bird is in the desirable range of the target
        if abs(error_z) < self.desirable_range_z:
            self.leveling_state = True
        else:
            self.leveling_state = False
        if abs(error_yaw) < self.desirable_range_yaw:
            self.directing_state_state = True
        else:
            self.directing_state = False

        self.get_logger().info(f"heyy: {steering_command}")

        self.publish_controls(speed_command, steering_command)

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

    def publish_target_marker(self):
        # Create a marker for the target level
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_level"
        marker.id = 0
        marker.type = Marker.ARROW  # Represent the target level as a cube
        marker.action = Marker.ADD
        marker.scale.x = 0.5  # Width of the marker
        marker.scale.y = 0.1  # Depth of the marker
        marker.scale.z = 0.1  # Height of the marker (thin layer for the level)

        # Set the color of the marker
        if self.leveling_state and self.directing_state:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green when in range of both
        elif self.leveling_state and not self.directing_state:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow when level but not directed
        elif self.leveling_state and not self.directing_state:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)  # Cyan when direccted but not level
        else:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # Magenta when neither

        # Set the position of the marker (fixed z, same x and y as the bird)
        marker.pose.position.x = self.current_pose.pose.position.x
        marker.pose.position.y = self.current_pose.pose.position.y
        marker.pose.position.z = self.target_z

        # First, rotate by 90 degrees around Y axis (to point along X axis in XY plane)
        initial_orientation = tf_transformations.quaternion_from_euler(0, 0, 0)  # 90 degrees around Y-axis
        
        # Then, apply the yaw around the Z-axis
        yaw_rotation = tf_transformations.quaternion_from_euler(0, 0, self.target_yaw)
        
        # Multiply both quaternions (initial * yaw) to get the final orientation
        final_orientation = tf_transformations.quaternion_multiply(yaw_rotation, initial_orientation)

        # No rotation for the marker
        marker.pose.orientation.x = final_orientation[0]
        marker.pose.orientation.y = final_orientation[1]
        marker.pose.orientation.z = final_orientation[2]
        marker.pose.orientation.w = final_orientation[3]

        # Publish the marker
        self.target_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelSwitching()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
