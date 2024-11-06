import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initial position and direction of the arrow
start_point = np.array([0, 0, 0])
direction = np.array([1, 1, 0])

# Set the limits of the plot
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])

# Create the quiver
quiver = ax.quiver(*start_point, *direction, color='b', length=1)

# Define the update function for the animation
def update_quiver(num, quiver, start_point, direction):
    # Modify the start point and direction for each frame
    # Example: rotate the arrow around the z-axis
    angle = np.deg2rad(num * 10)
    new_direction = np.array([
        np.cos(angle) * direction[0] - np.sin(angle) * direction[1],
        np.sin(angle) * direction[0] + np.cos(angle) * direction[1],
        direction[2]
    ])
    
    # Clear the old quiver and plot the updated one
    ax.collections.clear()  # Remove the old quiver
    ax.quiver(*start_point, *new_direction, color='b', length=1)

# Animate the quiver
ani = FuncAnimation(fig, update_quiver, fargs=(quiver, start_point, direction), frames=36, interval=100)

# Display the animation
plt.show()
