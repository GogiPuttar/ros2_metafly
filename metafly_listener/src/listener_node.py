#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import struct
import tf2_ros
import tf_transformations  # For quaternion and transformation calculations
from geometry_msgs.msg import TransformStamped
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class UdpListener(Node):
    def __init__(self):
        super().__init__('udp_listener')

        # Declare a parameter for the bird's name
        self.declare_parameter('bird_name', 'charlie_2')

        # Load the pose offsets from YAML
        package_share_directory = get_package_share_directory('metafly_listener')
        yaml_file_path = os.path.join(package_share_directory, 'config', 'pose_offsets.yaml')
        self.pose_offsets = self.load_yaml(yaml_file_path)

        # Get the bird's name and corresponding offset
        bird_name = self.get_parameter('bird_name').get_parameter_value().string_value
        self.offset = self.pose_offsets.get(bird_name)

        if self.offset is None:
            self.get_logger().error(f"No offset found for bird: {bird_name}")
            raise ValueError(f"No offset found for bird: {bird_name}")

        self.get_logger().info(f"Using offset for bird: {bird_name} - {self.offset}")

        # Create a publisher for the PoseStamped messages
        self.pose_publisher = self.create_publisher(PoseStamped, 'metafly_pose', 10)
        
        # Create a broadcaster for the TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Set up the UDP server
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)  # Reduce buffer size
        self.udp_socket.setblocking(False)
        self.udp_socket.bind(("0.0.0.0", 54321))

        # Set a timer to check for new messages and broadcast at 100Hz
        self.create_timer(1/100.0, self.timer_callback)

    def load_yaml(self, file_path):
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)

    def invert_offset(self, position, orientation):
        """
        Inverts the position and orientation offsets, separately
        Does not invert the combined transform of the position and orientation
        """
        # Convert position and orientation to transformation matrix
        trans_mat = tf_transformations.quaternion_matrix((
            orientation['x'], orientation['y'], orientation['z'], orientation['w']))

        # Invert the transformation matrix
        inverted_trans_mat = tf_transformations.inverse_matrix(trans_mat)

        # Invert the position offset, in the world frame
        inverted_position = [-position['x'], 
                             -position['y'],
                             -position['z']
                             ]

        # Extract the inverted orientation
        inverted_orientation = tf_transformations.quaternion_from_matrix(inverted_trans_mat)

        # return inverted_position, inverted_orientation
        return inverted_position, inverted_orientation

    def timer_callback(self):
        try:
            # Receive data from UDP socket
            data, addr = self.udp_socket.recvfrom(1024)  # Buffer size of 1024 bytes
            message = data.decode('utf-8')
            self.get_logger().debug(f"Received message from {addr}: {message}")

            # Parse the received message
            message_parts = message.split(", ")
            x, y, z = float(message_parts[1]), float(message_parts[2]), float(message_parts[3])
            qx, qy, qz, qw = float(message_parts[4]), float(message_parts[5]), float(message_parts[6]), float(message_parts[7])

            # Apply the inverted offset to the received pose
            received_position = {'x': x, 'y': y, 'z': z}
            received_orientation = {'x': qx, 'y': qy, 'z': qz, 'w': qw}

            offset_position = self.offset['position']
            offset_orientation = self.offset['orientation']
            inverted_offset_position, inverted_offset_orientation = self.invert_offset(offset_position, offset_orientation)

            # Adjust the position and orientation
            adjusted_position = {
                'x': received_position['x'] + inverted_offset_position[0],
                'y': received_position['y'] + inverted_offset_position[1],
                'z': received_position['z'] + inverted_offset_position[2],
            }
            adjusted_orientation = tf_transformations.quaternion_multiply(
                (qx, qy, qz, qw), inverted_offset_orientation)

            # Publish the adjusted PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = adjusted_position['x']
            pose_msg.pose.position.y = adjusted_position['y']
            pose_msg.pose.position.z = adjusted_position['z']
            pose_msg.pose.orientation.x = adjusted_orientation[0]
            pose_msg.pose.orientation.y = adjusted_orientation[1]
            pose_msg.pose.orientation.z = adjusted_orientation[2]
            pose_msg.pose.orientation.w = adjusted_orientation[3]
            self.pose_publisher.publish(pose_msg)

            # Broadcast transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = "metafly_base"
            t.transform.translation.x = adjusted_position['x']
            t.transform.translation.y = adjusted_position['y']
            t.transform.translation.z = adjusted_position['z']
            t.transform.rotation.x = adjusted_orientation[0]
            t.transform.rotation.y = adjusted_orientation[1]
            t.transform.rotation.z = adjusted_orientation[2]
            t.transform.rotation.w = adjusted_orientation[3]
            self.tf_broadcaster.sendTransform(t)

        except socket.timeout:
            self.get_logger().warn('No data received, retrying...')
        except Exception as e:
            self.get_logger().debug(f"Error receiving data: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = UdpListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
