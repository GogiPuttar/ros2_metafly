import pickle
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations  # For quaternion to Euler conversion

# Load the pickle file
filename = 'sessions/session_9/run_1.pkl'
with open(filename, 'rb') as file:
    data = pickle.load(file)

# Extract data
pose_data = data['pose']
controls_data = data['controls']
timestamps = np.array(data['timestamps'])  # Convert to NumPy array for easier manipulation

# Extract x, y, z, and quaternion (qx, qy, qz, qw) from pose data
x = [pose.pose.position.x for pose in pose_data]
y = [pose.pose.position.y for pose in pose_data]
z = [pose.pose.position.z for pose in pose_data]
qx = [pose.pose.orientation.x for pose in pose_data]
qy = [pose.pose.orientation.y for pose in pose_data]
qz = [pose.pose.orientation.z for pose in pose_data]
qw = [pose.pose.orientation.w for pose in pose_data]

# Convert quaternions to roll, pitch, and yaw
roll, pitch, yaw = [], [], []
for i in range(len(qx)):
    quaternion = [qx[i], qy[i], qz[i], qw[i]]
    euler = tf_transformations.euler_from_quaternion(quaternion)
    roll.append(euler[0])
    pitch.append(euler[1])
    yaw.append(euler[2])

# Extract speed and steering data from controls
speed = [controls.speed for controls in controls_data]
steering = [controls.steering for controls in controls_data]

# Plot x vs y
plt.figure()
plt.plot(x, y, label='Y vs X')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.xlim((-3.0, 1.3))
plt.ylim((-1.1, 1.7))
plt.title('Trajectory trace')
plt.legend()
plt.grid(True)

# Plot x, y, z vs time
plt.figure()
plt.plot(timestamps, x, label='X')
plt.plot(timestamps, y, label='Y')
plt.plot(timestamps, z, label='Z')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Position (X, Y, Z) vs Time')
plt.legend()
plt.grid(True)

# Plot roll, pitch, yaw vs time
plt.figure()
plt.plot(timestamps, roll, label='Roll')
plt.plot(timestamps, pitch, label='Pitch')
plt.plot(timestamps, yaw, label='Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Orientation (Roll, Pitch, Yaw) vs Time')
plt.legend()
plt.grid(True)

# Plot cmd_controls (speed, steering) vs time
plt.figure()
plt.plot(timestamps, speed, label='Speed')
plt.plot(timestamps, steering, label='Steering')
plt.xlabel('Time (s)')
plt.ylabel('Control Commands')
plt.title('Commanded Controls (Speed, Steering) vs Time')
plt.legend()
plt.grid(True)

# Show the plots
plt.show()

"""
Notes about runs/sessions:
Session Run Info

"""
