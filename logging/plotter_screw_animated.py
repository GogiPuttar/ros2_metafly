import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import least_squares
from matplotlib.animation import FuncAnimation

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
def calculate_pitch(z, radii):
    arc_length = np.cumsum(np.hypot(np.diff(radii), np.diff(z)))
    total_z_change = z[-1] - z[0]
    num_rotations = arc_length[-1] / (2 * np.pi * np.mean(radii))
    pitch = total_z_change / num_rotations
    return pitch

# Load the pickle file
filename = 'sessions/session_67/run_6.pkl'
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

# Constraints for uniform scaling
x_constraints = [-3.7, 3.7]  # [m, m]
y_constraints = [-2.5, 1.7]  # [m, m]
z_constraints = [0.0, 2.7]   # [m, m]

# Create figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(x_constraints[0], x_constraints[1])
ax.set_ylim(y_constraints[0], y_constraints[1])
ax.set_zlim(z_constraints[0], z_constraints[1])
ax.set_box_aspect([abs(x_constraints[1] - x_constraints[0]),
                   abs(y_constraints[1] - y_constraints[0]),
                   abs(z_constraints[1] - z_constraints[0])])

# Set axis labels and title
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Animated 3D Screw Motion of Bird\'s Trajectory')

# Initial plot setup for trajectory, center point, and vector
trajectory_line, = ax.plot([], [], [], 'b-', label='Flight Path')
center_point, = ax.plot([], [], 'go', label='Screw Center')
vector_line, = ax.plot([], [], 'r-', label='Vector to Center')
additional_vector, = ax.plot([], [], color='orange', linestyle='-', label='Control Vector')
arrowhead_left, = ax.plot([], [], [], color='orange', linewidth=2)
arrowhead_right, = ax.plot([], [], [], color='orange', linewidth=2)

# Add text box displaying radius and pitch information
textstr = f'Radius: {radius:.2f} m\nPitch: {pitch:.2f} m/rotation'
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
ax.text2D(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
          verticalalignment='top', bbox=props)

target_x, target_y = 0, 0  # Target point in the XY plane

# Store the cumulative arc length
cumulative_arc_length = [0]

def update(frame):
    global quiver
    # Update the trajectory line with points up to the current frame
    trajectory_line.set_data(x_filtered[:frame], y_filtered[:frame])
    trajectory_line.set_3d_properties(z_filtered[:frame])

    # Define the fixed-size window for fitting the circle (e.g., 50 frames)
    window_size = 50
    start_index = max(0, frame - window_size)

    # Fit the circle if there are enough points in the window
    if frame - start_index >= 3:
        xc, yc, _ = fit_circle(x_filtered[start_index:frame], y_filtered[start_index:frame])
        
        # Calculate distance from the center to the current point
        current_x, current_y = x_filtered[frame], y_filtered[frame]
        distance_to_center = np.sqrt((current_x - xc)**2 + (current_y - yc)**2)
        
        # Enforce maximum distance of 4 meters
        max_radius = 3.0
        if distance_to_center > max_radius:
            # Scale the position to lie exactly 4 meters away from the center
            scale_factor = max_radius / distance_to_center
            # current_x = xc + (current_x - xc) * scale_factor
            # current_y = yc + (current_y - yc) * scale_factor

            xc = current_x + (xc - current_x) * scale_factor
            yc = current_y + (yc - current_y) * scale_factor

            # print(scale_factor)

        # Update the center point and vector line based on the adjusted point
        center_point.set_data([xc], [yc])
        center_point.set_3d_properties([0])  # Project on the XY plane at z = 0

        # Update the vector line from target to the current center
        vector_line.set_data([target_x, xc], [target_y, yc])
        vector_line.set_3d_properties([0, 0])  # Keep vector in XY plane

        # Calculate the new vector's orientation and length
        dx = xc - current_x
        dy = yc - current_y
        target_dx = target_x - xc
        target_dy = target_y - yc

        # Length of the new vector based on the dot product
        length = dx * target_dx + dy * target_dy
        # length = 0 # to visualize without arrow

        # Normalize the direction vector and scale it by the computed length
        direction_norm = np.sqrt(dx**2 + dy**2)
        unit_dx = dx / direction_norm
        unit_dy = dy / direction_norm
        end_x = xc + length * unit_dx
        end_y = yc + length * unit_dy

        # Determine arrow color based on length
        arrow_color = 'orange' if length > 0 else 'purple'

        # Update the additional vector line starting from (xc, yc)
        additional_vector.set_data([xc, end_x], [yc, end_y])
        additional_vector.set_3d_properties([0, 0])  # Keep it in the XY plane
        additional_vector.set_color(arrow_color)

        # Calculate and plot the arrowhead
        arrowhead_length = 0.1 * length  # Adjust the size of the arrowhead as needed
        angle_offset = np.pi / 8  # Angle for arrowhead rotation
        cos_angle, sin_angle = np.cos(angle_offset), np.sin(angle_offset)

        # Rotate the vector for each arrowhead line segment
        arrow_left_dx = cos_angle * unit_dx - sin_angle * unit_dy
        arrow_left_dy = sin_angle * unit_dx + cos_angle * unit_dy
        arrow_right_dx = cos_angle * unit_dx + sin_angle * unit_dy
        arrow_right_dy = -sin_angle * unit_dx + cos_angle * unit_dy

        # Arrowhead line endpoints
        arrow_left_x = end_x - arrowhead_length * arrow_left_dx
        arrow_left_y = end_y - arrowhead_length * arrow_left_dy
        arrow_right_x = end_x - arrowhead_length * arrow_right_dx
        arrow_right_y = end_y - arrowhead_length * arrow_right_dy

        # Update the arrowhead line segments
        arrowhead_left.set_data([end_x, arrow_left_x], [end_y, arrow_left_y])
        arrowhead_left.set_3d_properties([0, 0])
        arrowhead_left.set_color(arrow_color)
        arrowhead_right.set_data([end_x, arrow_right_x], [end_y, arrow_right_y])
        arrowhead_right.set_3d_properties([0, 0])
        arrowhead_right.set_color(arrow_color)

    else:
        center_point.set_data([], [])
        vector_line.set_data([], [])
        additional_vector.set_data([], [])
        arrowhead_left.set_data([], [])
        arrowhead_right.set_data([], [])

    return [trajectory_line, center_point, vector_line, additional_vector, arrowhead_left, arrowhead_right]

# Animate with frames progressing over the length of the trajectory
ani = FuncAnimation(fig, update, frames=len(x_filtered), interval=20, blit=True)

plt.legend()
plt.show()
