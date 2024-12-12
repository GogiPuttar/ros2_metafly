import pickle
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from alphashape import alphashape
import trimesh

# Load the pickle file
filename = 'sessions/session_70/run_1.pkl'
with open(filename, 'rb') as file:
    data = pickle.load(file)

# Constraints for plotting
# x_constraints = [-3.0, 1.3]  # [m, m]
# y_constraints = [-1.1, 1.7]  # [m, m]
# z_constraints = [0.0, 2.0]   # [m, m]

# Old
# x_constraints = [-5.2, 2.1]  # [m, m]
# y_constraints = [-1.8, 2.3]  # [m, m]
# z_constraints = [0.0, 2.7]   # [m, m]
    
# Post callibration on Oct 15
x_constraints = [-3.7, 3.7]  # [m, m]
y_constraints = [-2.5, 1.7]  # [m, m]
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

# Create an interactive 3D plot using Plotly
fig = go.Figure()

# Calculate center for the x and y, and set z to 0
center_x = (x_constraints[0] + x_constraints[1]) / 2
center_y = (y_constraints[0] + y_constraints[1]) / 2
center_focus = dict(x=0, y=0, z=-0.2)

# Add the alpha shape mesh with thin black edges
fig.add_trace(go.Mesh3d(
    x=vertices[:, 0],
    y=vertices[:, 1],
    z=vertices[:, 2],
    i=faces[:, 0],
    j=faces[:, 1],
    k=faces[:, 2],
    opacity=0.45,
    color='lightblue',
    name='Alpha Shape',
    showscale=False,
    flatshading=True,
    contour=dict(color='black', width=10)  # Thin black lines along the edges
))

# Add the vertices as scatter points with a smaller size
fig.add_trace(go.Scatter3d(
    x=vertices[:, 0],
    y=vertices[:, 1],
    z=vertices[:, 2],
    mode='markers',
    marker=dict(color='blue', size=0.5),  # Smaller size for the points
    name='Vertices'
))

# Set axis properties with equal scaling for all axes
fig.update_layout(
    scene=dict(
        xaxis=dict(
            title='X Position (m)',
            range=x_constraints,
            showspikes=False,
            backgroundcolor='black',
            gridcolor='white',  # White grid lines
            zerolinecolor='white',  # White zero line
            color='white'  # White axis labels
        ),
        yaxis=dict(
            title='Y Position (m)',
            range=y_constraints,
            showspikes=False,
            backgroundcolor='black',
            gridcolor='white',  # White grid lines
            zerolinecolor='white',  # White zero line
            color='white'  # White axis labels
        ),
        zaxis=dict(
            title='Z Position (m)',
            range=z_constraints,
            showspikes=False,
            backgroundcolor='black',
            gridcolor='white',  # White grid lines
            zerolinecolor='white',  # White zero line
            color='white'  # White axis labels
        ),
        aspectmode='data',  # Ensures equal scaling of axes
        camera=dict(center=center_focus)  # Set camera center
    ),
    title='Interactive 3D Surface of the Observable Workspace',
    margin=dict(l=0, r=0, b=0, t=40),
    paper_bgcolor='black',  # Set the surrounding background color to black
    font=dict(color='white'),  # Set the font color to white for better contrast
    dragmode='orbit'
)

# Save the plot as an HTML file
fig.write_html('3d_plot_interactive_draggable.html')

print("Interactive 3D plot saved as '3d_plot_interactive.html'")

# # Save the plot as an HTML file with custom JavaScript for wobbling effect
# html_content = fig.to_html(full_html=False)

# # JavaScript for wobbling effect on scroll
# js_script = """
# <script>
# document.addEventListener('scroll', function() {
#     var scrollTop = window.pageYOffset || document.documentElement.scrollTop;
#     var wobbleAngle = 0.01 * scrollTop; // Adjust this factor for more/less wobble

#     // Update the camera with slight wobble effect
#     Plotly.relayout('plotly-graph', {
#         'scene.camera.eye': {
#             x: 1.5 * Math.cos(wobbleAngle),
#             y: 1.5 * Math.sin(wobbleAngle),
#             z: 1.5
#         }
#     });
# });
# </script>
# """

# # Combine the HTML and JavaScript
# with open('3d_plot_interactive_wobble.html', 'w') as f:
#     f.write(html_content.replace('</body>', js_script + '</body>'))

# print("Interactive 3D plot with wobble effect saved as '3d_plot_interactive_wobble.html'")

# Save the plot as an HTML file with custom JavaScript for spinning effect
html_content = fig.to_html(full_html=False)

# JavaScript for spinning effect
js_script = """
<script>
let spinning = true;

// Function to animate the plot rotation
function spinPlot() {
    if (spinning) {
        // Get current time to calculate rotation
        let currentTime = new Date().getTime() / 1000;
        let rotationAngle = 0.1 * currentTime; // Adjust this factor for speed

        // Update the camera to spin the plot
        Plotly.relayout('plotly-graph', {
            'scene.camera.eye': {
                x: 2 * Math.cos(rotationAngle),
                y: 2 * Math.sin(rotationAngle),
                z: 1.5
            }
        });

        requestAnimationFrame(spinPlot); // Continue the animation
    }
}

// Start the spinning animation
spinPlot();

// Stop spinning when the plot is clicked
document.getElementById('plotly-graph').addEventListener('click', function() {
    spinning = false;
});
</script>
"""

# Combine the HTML and JavaScript
with open('3d_plot_interactive_spin.html', 'w') as f:
    f.write(html_content.replace('</body>', js_script + '</body>'))

print("Interactive 3D plot with spinning effect saved as '3d_plot_interactive_spin.html'")

plt.show()

"""
Notes about runs/sessions:
Session Run Info
11      1   Scan of workspace. Less focus on the floor
14      1   Scan of workspace after callibration on Oct 15
66      1   Scan of workspace after adding 6 cameras + "poor" callibration on Nov 3
70      1   Scan of workspace on Dec 7
"""