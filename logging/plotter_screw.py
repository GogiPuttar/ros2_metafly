import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.optimize import least_squares

# Function to fit a circle to the XY trajectory
def fit_circle(x, y):
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

# Function to calculate the pitch of the screw motion
def calculate_pitch(z, radii, full_rotation=2 * np.pi):
    arc_length = np.cumsum(np.hypot(np.diff(radii), np.diff(z)))
    total_z_change = z[-1] - z[0]
    num_rotations = arc_length[-1] / (2 * np.pi * np.mean(radii))
    pitch = total_z_change / num_rotations
    return pitch

# Function to create a cylinder for the screw motion visualization
def create_cylinder(xc, yc, radius, pitch, height, num_points=100):
    theta = np.linspace(0, 2 * np.pi, num_points)
    z = np.linspace(0, height, num_points)
    X = xc + radius * np.cos(theta)
    Y = yc + radius * np.sin(theta)
    Z = np.tile(z, (num_points, 1)).T
    return X, Y, Z

# Load the pickle file
filename = 'sessions/session_23/run_8.pkl'
with open(filename, 'rb') as file:
    data = pickle.load(file)

# Extract data
pose_data = data['pose']
controls_data = data['controls']

# Extract x, y, z, and speed data
x = np.array([pose.pose.position.x for pose in pose_data])
y = np.array([pose.pose.position.y for pose in pose_data])
z = np.array([pose.pose.position.z for pose in pose_data])
speed = np.array([controls.speed for controls in controls_data])

# Filter data to keep only the points inside the workspace (non-zero speed)
in_workspace = speed != 0
x_filtered = x[in_workspace]
y_filtered = y[in_workspace]
z_filtered = z[in_workspace]

# Fit a circular arc to the XY-trajectory
xc, yc, radius = fit_circle(x_filtered, y_filtered)

# Calculate the pitch of the screw motion
pitch = calculate_pitch(z_filtered, np.sqrt((x_filtered - xc) ** 2 + (y_filtered - yc) ** 2))

# Create the screw motion cylinder for visualization
X, Y, Z = create_cylinder(xc, yc, radius, pitch, height=pitch)

# Constraints for uniform scaling
x_constraints = [-3.7, 3.7]  # [m, m]
y_constraints = [-2.5, 1.7]  # [m, m]
z_constraints = [0.0, 2.7]   # [m, m]

# 3D Plot: Bird's Trajectory and Screw Motion
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the actual trajectory
ax.plot(x_filtered, y_filtered, z_filtered, label='Flight Path', color='blue')

# Plot the cylinder (screw motion), green if pitch is positive, red if pitch is negative
cylinder_color = 'green' if pitch >= 0 else 'red'
ax.plot_surface(X, Y, Z, color=cylinder_color, alpha=0.5)

# Set axis labels and title
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Screw Motion Fit to Bird\'s Trajectory')

# Set the exact axis limits based on workspace constraints
ax.set_xlim(x_constraints[0], x_constraints[1])
ax.set_ylim(y_constraints[0], y_constraints[1])
ax.set_zlim(z_constraints[0], z_constraints[1])

ax.set_box_aspect([abs(x_constraints[1] - x_constraints[0]),
                   abs(y_constraints[1] - y_constraints[0]),
                   abs(z_constraints[1] - z_constraints[0])])

# Add text box displaying radius and pitch information
textstr = f'Radius: {radius:.2f} m\nPitch: {pitch:.2f} m/rotation'
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
ax.text2D(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
          verticalalignment='top', bbox=props)

plt.legend()
plt.show()
