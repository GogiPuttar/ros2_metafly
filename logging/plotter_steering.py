import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from collections import defaultdict

# Define the bad runs for each session
bad_runs = {
    'session_17': ['run_1', 'run_4', 'run_5'],  
    'session_18': ['run_4'],           
    'session_19': ['run_2', 'run_3', 'run_4', 'run_6' 'run_7'],           
    'session_20': ['run_2', 'run_3', 'run_4', 'run_5', 'run_7', 'run_8', 'run_9', 'run_10', 'run_11', 'run_13', 'run_15'],           
    'session_23': ['run_1', 'run_3', 'run_5', 'run_6', 'run_7', 'run_9', 'run_10'],           
    'session_24': ['run_1', 'run_2', 'run_5', 'run_7', 'run_8', 'run_10'],           
    'session_25': ['run_2', 'run_5', 'run_7'],           
    'session_27': ['run_2', 'run_3', 'run_6', 'run_7', 'run_8', 'run_9', 'run_12', 'run_13', 'run_14', 'run_15'],           
    'session_28': ['run_6', 'run_7', 'run_8'],           
    'session_30': ['run_3', 'run_5'],           
    'session_31': ['run_6', 'run_7', 'run_10', 'run_11', 'run_12', 'run_13', 'run_12', 'run_13', 'run_18', 'run_19', 'run_20'],   
    'session_33': ['run_1', 'run_2', 'run_4', 'run_5', 'run_7', 'run_9'],
    'session_36': ['run_4', 'run_5', 'run_9'],
    'session_39': ['run_1', 'run_4', 'run_5', 'run_6', 'run_7', 'run_8'],
    'session_40': ['run_3'],
    'session_41': ['run_3', 'run_4', 'run_6', 'run_7', 'run_8'],
    'session_44': ['run_2', 'run_4', 'run_8', 'run_9'],
    'session_45': ['run_4', 'run_7', 'run_8', 'run_11', 'run_12', 'run_14', 'run_15', 'run_16'],
    'session_48': ['run_1', 'run_2', 'run_3'],
    'session_49': ['run_2', 'run_4', 'run_7'],
    'session_50': ['run_1', 'run_2', 'run_4', 'run_7', 'run_8', 'run_12', 'run_13'],        
    # Add more sessions and runs here as needed
}

# Define the valid sessions to process (if empty, process all sessions)
valid_sessions = ['session_16', 
                  'session_17', 
                  'session_18', 
                  'session_19', 
                  'session_20', 
                  'session_23', 
                  'session_24', 
                  'session_25', 
                  'session_27', 
                  'session_28', 
                  'session_30',
                  'session_31',
                  'session_33',
                  'session_36',
                  'session_38',
                  'session_39',
                  'session_40',
                  'session_41',
                  'session_44',
                  'session_45',
                  'session_48',
                  'session_49',
                  'session_50',
                  ]  # Specify only the sessions you want to process

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
    return radius

# Function to calculate the pitch of the screw motion
def calculate_pitch(z, radii, full_rotation=2 * np.pi):
    arc_length = np.cumsum(np.hypot(np.diff(radii), np.diff(z)))
    total_z_change = z[-1] - z[0]
    num_rotations = arc_length[-1] / (2 * np.pi * np.mean(radii))
    pitch = total_z_change / num_rotations
    return pitch

# Directory containing all sessions and runs
data_dir = 'sessions'

# Lists to store turning radii and pitches for each steering command
turning_radii = []
pitches = []
steering_commands = []

# Loop through each session and each run
for session in os.listdir(data_dir):
    if valid_sessions and session not in valid_sessions:
        continue  # Skip this session if it's not in the valid sessions list

    session_path = os.path.join(data_dir, session)

    # Skip bad runs in this session
    bad_run_list = bad_runs.get(session, [])

    for run in os.listdir(session_path):
        run_name = run.split('.')[0]  # Extract run name without file extension
        if run_name in bad_run_list:
            continue  # Skip the bad run

        run_path = os.path.join(session_path, run)

        # print(run_path)

        # Load the pickle file for the run
        with open(run_path, 'rb') as file:
            data = pickle.load(file)

        # Extract data
        pose_data = data['pose']
        controls_data = data['controls']

        # Extract x, y, z, and steering command data
        x = np.array([pose.pose.position.x for pose in pose_data])
        y = np.array([pose.pose.position.y for pose in pose_data])
        z = np.array([pose.pose.position.z for pose in pose_data])
        average_sign = np.mean([controls.steering for controls in controls_data])
        if average_sign < 0:
            steering_command = np.min([controls.steering for controls in controls_data])
        elif average_sign > 0:
            steering_command = np.max([controls.steering for controls in controls_data])
        else:
            steering_command = 0

        # Filter data to keep only the points inside the workspace (non-zero speed)
        speed = np.array([controls.speed for controls in controls_data])
        in_workspace = speed != 0
        x_filtered = x[in_workspace]
        y_filtered = y[in_workspace]
        z_filtered = z[in_workspace]

        # Fit a circular arc to the XY-trajectory and calculate turning radius
        turning_radius = fit_circle(x_filtered, y_filtered)

        # Calculate the pitch of the screw motion
        pitch = calculate_pitch(z_filtered, np.sqrt((x_filtered - np.mean(x_filtered)) ** 2 + 
                                                    (y_filtered - np.mean(y_filtered)) ** 2))

        # Append the results to the lists
        turning_radii.append(turning_radius)
        pitches.append(pitch)
        steering_commands.append(steering_command)

# Convert lists to numpy arrays for plotting
turning_radii = np.array(turning_radii)
pitches = np.array(pitches)
steering_commands = np.array(steering_commands)

# Group data by unique steering commands
grouped_turning_radii = defaultdict(list)
grouped_pitches = defaultdict(list)
unique_steering_commands = np.unique(steering_commands)

# Group the data for each unique steering command
for i, steering_command in enumerate(steering_commands):
    grouped_turning_radii[steering_command].append(turning_radii[i])
    grouped_pitches[steering_command].append(pitches[i])

# Plot 1/turning radius vs steering command
plt.figure()
for steering_command in unique_steering_commands:
    # Extract the turning radii for the current steering command
    radii = np.array(grouped_turning_radii[steering_command])
    
    # Scatter plot the data points
    plt.scatter([steering_command] * len(radii), 1 / radii, alpha=0.6, color='blue', label='1/Turning Radius')

    # Plot line segments connecting the points
    plt.plot([steering_command] * len(radii), 1 / radii, color='blue', linestyle='-', alpha=0.5)
    
    # Compute and plot the mean for the group
    mean_radius = np.mean(1 / radii)
    plt.scatter(steering_command, mean_radius, color='orange', marker='D', label='Mean 1/Turning Radius')

plt.xlabel('Steering Command')
plt.ylabel('1/Turning Radius (1/m)')
plt.title('1/Turning Radius vs Steering Command')
plt.grid(True)
# plt.legend()

# Plot 1/turning radius vs steering command
# plt.figure()
# for steering_command in unique_steering_commands:
#     # Extract the turning radii for the current steering command
#     radii = np.array(grouped_turning_radii[steering_command])
    
#     # Scatter plot the data points
#     plt.scatter([steering_command] * len(radii), radii, color='black', label='1/Turning Radius')

#     # Plot line segments connecting the points
#     plt.plot([steering_command] * len(radii), radii, color='black', linestyle='-', alpha=0.5)
    
#     # Compute and plot the mean for the group
#     mean_radius = np.mean(radii)
#     plt.scatter(steering_command, mean_radius, color='orange', marker='D', label='Mean 1/Turning Radius')

# plt.xlabel('Steering Command')
# plt.ylabel('1/Turning Radius (1/m)')
# plt.title('1/Turning Radius vs Steering Command')
# plt.grid(True)

# Plot pitch vs steering command
plt.figure()
for steering_command in unique_steering_commands:
    # Extract the pitches for the current steering command
    pitch_values = np.array(grouped_pitches[steering_command])

    # Scatter plot the data points
    plt.scatter([steering_command] * len(pitch_values), pitch_values, alpha=0.6, color='red', label='Pitch')

    # Plot line segments connecting the points
    plt.plot([steering_command] * len(pitch_values), pitch_values, color='red', linestyle='-', alpha=0.5)

    # Compute and plot the mean for the group
    mean_pitch = np.mean(pitch_values)
    plt.scatter(steering_command, mean_pitch, color='green', marker='D', label='Mean Pitch')

# Add a horizontal line at y=0 (black dotted line)
plt.axhline(0, color='black', linestyle='--', linewidth=1.5, label='y=0')

plt.xlabel('Steering Command')
plt.ylabel('Pitch (m/rotation)')
plt.title('Pitch vs Steering Command')
plt.grid(True)
# plt.legend()

plt.show()