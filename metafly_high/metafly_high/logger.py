#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from metafly_interfaces.msg import Controls  # Import the Controls message
import os
import pickle
import time
import threading  # For key press detection
from datetime import datetime

class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        self.get_logger().info('Logger node initialized')

        # Subscriber to the Pose data
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'metafly_pose',
            self.pose_callback,
            10
        )

        # Subscriber to the Controls data
        self.controls_subscriber = self.create_subscription(
            Controls,
            'cmd_controls',
            self.controls_callback,
            10
        )

        # Variables for storing data and recording state
        self.recording = False
        self.data = {'pose': [], 'controls': [], 'timestamps': []}
        self.start_time = None
        self.session_dir = self.create_session_directory()

        # Thread for listening to key presses (starts automatically)
        self.key_press_thread = threading.Thread(target=self.key_listener)
        self.key_press_thread.daemon = True
        self.key_press_thread.start()

        self.run_count = 1  # Counter for run files within each session

        self.current_controls = Controls()

    def create_session_directory(self):
        """Creates a new session directory for storing recording data."""
        base_dir = os.path.join(os.getcwd(), "sessions")
        os.makedirs(base_dir, exist_ok=True)

        session_num = 1
        while os.path.exists(os.path.join(base_dir, f"session_{session_num}")):
            session_num += 1

        session_dir = os.path.join(base_dir, f"session_{session_num}")
        os.makedirs(session_dir)
        self.get_logger().info(f"Session directory created: {session_dir}")
        return session_dir

    def key_listener(self):
        """Listens for key presses to start/stop recording."""
        while True:
            key = input("Press 's' to start/stop recording: ").strip().lower()
            if key == 's':
                if self.recording:
                    self.stop_recording()
                else:
                    self.start_recording()

    def start_recording(self):
        """Starts the recording of data."""
        self.get_logger().info("Recording started...")
        self.recording = True
        self.start_time = time.time()

    def stop_recording(self):
        """Stops the recording of data and saves it to a .pkl file."""
        self.get_logger().info("Recording stopped...")
        self.recording = False
        # Save the recorded data to a .pkl file
        self.save_data()

        # Reset the data for the next run
        self.data = {'pose': [], 'controls': [], 'timestamps': []}
        self.run_count += 1

    def save_data(self):
        """Saves the collected data as a .pkl file."""
        filename = f"run_{self.run_count}.pkl"
        filepath = os.path.join(self.session_dir, filename)
        with open(filepath, 'wb') as f:
            pickle.dump(self.data, f)
        self.get_logger().info(f"Data saved to {filepath}")

    def pose_callback(self, msg):
        """Callback for the pose data."""
        if self.recording:
            self.data['pose'].append(msg)
            self.data['timestamps'].append(time.time() - self.start_time)
            self.data['controls'].append(self.current_controls)

    def controls_callback(self, msg):
        """Callback for the controls data."""
        self.current_controls = msg
        # if self.recording:
        #     self.data['controls'].append(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
