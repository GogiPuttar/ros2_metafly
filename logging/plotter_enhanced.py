import pickle
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations  # For quaternion to Euler conversion
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting

# Load the pickle file
filename = 'sessions/session_63/run_24.pkl'
with open(filename, 'rb') as file:
    data = pickle.load(file)

# Constraints for plotting
# x_constraints = [-3.0, 1.3]  # [m, m]
# y_constraints = [-1.1, 1.7]  # [m, m]
# z_constraints = [0.0, 2.0]   # [m, m]
# Constraints for uniform scaling
x_constraints = [-3.7, 3.7]  # [m, m]
y_constraints = [-2.5, 1.7]  # [m, m]
z_constraints = [0.0, 2.7]   # [m, m]

# Extract data
pose_data = data['pose']
controls_data = data['controls']
timestamps = np.array(data['timestamps'])  # Convert to NumPy array

# Extract x, y, z, and quaternion from pose data
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
speed = np.array([controls.speed for controls in controls_data])
steering = np.array([controls.steering for controls in controls_data])

# 3D plot: Bird's trajectory in XYZ with color-coding
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# colors = ['red' if (s == 0 and st == 0) else 'blue' for s, st in zip(speed, steering)]
colors = ['red' if (st == 0) else 'blue' for s, st in zip(speed, steering)]

for i in range(1, len(x)):
    ax.plot(x[i-1:i+1], y[i-1:i+1], z[i-1:i+1], color=colors[i-1])

# Plot the starting point (orange sphere)
ax.scatter(x[0], y[0], z[0], color='orange', s=100, label='Start Position', marker='o')

# Plot the end point (green cross)
ax.scatter(x[-1], y[-1], z[-1], color='green', s=100, label='End Position', marker='x')

ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Trajectory of Bird')
plt.grid(True)

ax.set_box_aspect([abs(x_constraints[1] - x_constraints[0]),
                   abs(y_constraints[1] - y_constraints[0]),
                   abs(z_constraints[1] - z_constraints[0])])

# Set axis limits based on constraints
ax.set_xlim(x_constraints)
ax.set_ylim(y_constraints)
ax.set_zlim(z_constraints)

# Target and actual Z vs time, with normalized speed command
target_z = 1.5  # Example target Z value
normalized_speed = speed / 127.0  # Normalize speed to [0, 1]

plt.figure()
plt.plot(timestamps, z, label='Actual Z', color='blue')
plt.plot(timestamps, [target_z] * len(timestamps), label='Target Z', linestyle='--', color='green')
plt.plot(timestamps, normalized_speed, label='Normalized Speed Command', linestyle='-.', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Z Position / Speed')
plt.title('Z Position and Speed Command vs Time')
plt.legend()
plt.grid(True)

# Target and actual Yaw vs time, with normalized steering command
target_yaw = 0.0  # Example target yaw value
normalized_steering = steering / 127.0  # Normalize steering to [-1, 1]

plt.figure()
plt.plot(timestamps, yaw, label='Actual Yaw', color='blue')
plt.plot(timestamps, [target_yaw] * len(timestamps), label='Target Yaw', linestyle='--', color='green')
plt.plot(timestamps, normalized_steering, label='Normalized Steering Command', linestyle='-.', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (rad) / Steering')
plt.title('Yaw and Steering Command vs Time')
plt.legend()
plt.grid(True)

# Show all plots
plt.show()

"""
Notes about runs/sessions:
Session Run Info

"""