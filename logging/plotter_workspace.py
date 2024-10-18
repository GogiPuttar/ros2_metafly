import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from alphashape import alphashape
import trimesh

# Load the pickle file
filename = 'sessions/session_11/run_1.pkl'
with open(filename, 'rb') as file:
    data = pickle.load(file)

# Constraints for plotting
# x_constraints = [-3.0, 1.3]  # [m, m]
# y_constraints = [-1.1, 1.7]  # [m, m]
# z_constraints = [0.0, 2.0]   # [m, m]

x_constraints = [-5.2, 2.1]  # [m, m]
y_constraints = [-1.8, 2.3]  # [m, m]
z_constraints = [0.0, 2.7]   # [m, m]

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
points = np.array([x[in_workspace], y[in_workspace], z[in_workspace]]).T

# Generate the alpha shape (a tight hull around the points)
alpha = 0.5  # Adjust alpha for a tighter or looser fit
alpha_shape = alphashape(points, alpha)

# Convert the alpha shape to a Trimesh object (if it isn't already)
if not isinstance(alpha_shape, trimesh.Trimesh):
    alpha_shape = trimesh.Trimesh(vertices=alpha_shape.vertices, faces=alpha_shape.faces)

# Extract vertices and faces for plotting
vertices = alpha_shape.vertices
faces = alpha_shape.faces

# 3D Surface Plot: Alpha Shape of the Workspace
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the alpha shape as a smooth 3D surface
mesh = Poly3DCollection(vertices[faces], alpha=0.3, edgecolor='k')
ax.add_collection3d(mesh)

# Plot the vertices of the alpha shape as green points
ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], c='green', s=10)

# Set axis labels and title
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Smooth 3D Surface of the Observable Workspace')

# Set equal scaling for all axes
ax.set_box_aspect([abs(x_constraints[1] - x_constraints[0]),
                   abs(y_constraints[1] - y_constraints[0]),
                   abs(z_constraints[1] - z_constraints[0])])

# Set axis limits
ax.set_xlim(x_constraints)
ax.set_ylim(y_constraints)
ax.set_zlim(z_constraints)

# Add grid and show the plot
plt.grid(True)
plt.show()

"""
Notes about runs/sessions:
Session Run Info
11      1   Scan of workspace. Less focus on the floor

"""