import yaml
import rclpy
import rosbag2_py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation as R
from imu_msgs.msg import IMUmsg  # Import your custom message

# Load configuration from YAML file
with open('config.yaml', 'r') as file:
    config = yaml.safe_load(file)

# Function to convert quaternion to Euler angles (Roll, Pitch, Yaw)
def quaternion_to_euler(qx, qy, qz, qw):
    rotation = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)
    return roll, pitch, yaw

# Function to extract data from custom IMUmsg message
def extract_imu_and_mag_data_from_bag(bag_file, topic):
    # Initialize the ROS bag reader
    storage = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    # Extract IMU and magnetometer data from the specified topic
    data = []
    while reader.has_next():
        (topic_name, message_bytes, t) = reader.read_next()
        if topic_name == topic:
            message = deserialize_message(message_bytes, IMUmsg)  # Deserialize the custom IMUmsg message
            header = message.header
            time_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
            
            # Extract IMU data from the imu field
            imu = message.imu
            mag_field = message.mag_field  # Magnetometer data

            data.append({
                'time_sec': time_sec,
                'linear_acceleration_x': imu.linear_acceleration.x,
                'linear_acceleration_y': imu.linear_acceleration.y,
                'linear_acceleration_z': imu.linear_acceleration.z,
                'angular_velocity_x': imu.angular_velocity.x,
                'angular_velocity_y': imu.angular_velocity.y,
                'angular_velocity_z': imu.angular_velocity.z,
                'orientation_x': imu.orientation.x,
                'orientation_y': imu.orientation.y,
                'orientation_z': imu.orientation.z,
                'orientation_w': imu.orientation.w,
                'magnetic_field_x': mag_field.magnetic_field.x,  # Magnetometer X
                'magnetic_field_y': mag_field.magnetic_field.y,  # Magnetometer Y
                'magnetic_field_z': mag_field.magnetic_field.z   # Magnetometer Z
            })
    
    return pd.DataFrame(data)

# Extract stationary IMU and magnetometer data
stationary_imu_df = extract_imu_and_mag_data_from_bag(config['stationary_data']['bag_file'], config['stationary_data']['topic'])

# Display extracted IMU data
print(stationary_imu_df.head())

# Convert quaternions to Euler angles
euler_angles = [quaternion_to_euler(qx, qy, qz, qw) for (qx, qy, qz, qw) in 
                zip(stationary_imu_df['orientation_x'], 
                    stationary_imu_df['orientation_y'], 
                    stationary_imu_df['orientation_z'], 
                    stationary_imu_df['orientation_w'])]
roll_data, pitch_data, yaw_data = zip(*euler_angles)

# Calculate and print statistics for accelerometer, gyroscope, Euler angles, and magnetometer
def print_statistics(data, label):
    mean_val = np.mean(data)
    std_val = np.std(data)
    print(f'{label} - Mean: {mean_val:.4f}, Standard Deviation: {std_val:.4f}')

# Accelerometer statistics
print_statistics(stationary_imu_df['linear_acceleration_x'], 'Linear Acceleration X')
print_statistics(stationary_imu_df['linear_acceleration_y'], 'Linear Acceleration Y')
print_statistics(stationary_imu_df['linear_acceleration_z'], 'Linear Acceleration Z')

# Gyroscope statistics
print_statistics(stationary_imu_df['angular_velocity_x'], 'Angular Velocity X')
print_statistics(stationary_imu_df['angular_velocity_y'], 'Angular Velocity Y')
print_statistics(stationary_imu_df['angular_velocity_z'], 'Angular Velocity Z')

# Euler angles statistics
print_statistics(roll_data, 'Roll Angle')
print_statistics(pitch_data, 'Pitch Angle')
print_statistics(yaw_data, 'Yaw Angle')

# Magnetometer statistics
print_statistics(stationary_imu_df['magnetic_field_x'], 'Magnetic Field X')
print_statistics(stationary_imu_df['magnetic_field_y'], 'Magnetic Field Y')
print_statistics(stationary_imu_df['magnetic_field_z'], 'Magnetic Field Z')

# Plotting time series data for Accelerometer, Gyroscope, Magnetometer, and Euler angles
time = stationary_imu_df['time_sec']

# Plot Magnetometer Data
plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['magnetic_field_x'], label='Magnetometer X', color='blue')
plt.title('Magnetometer Data Over Time (X direction)', fontsize=22)
plt.ylabel('Magnetic Field (T)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['magnetic_field_y'], label='Magnetometer Y', color='orange')
plt.title('Magnetometer Data Over Time (Y direction)', fontsize=22)
plt.ylabel('Magnetic Field (T)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['magnetic_field_z'], label='Magnetometer Z', color='green')
plt.title('Magnetometer Data Over Time (Z direction)', fontsize=22)
plt.ylabel('Magnetic Field (T)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

# Define the plot_histogram function to avoid the NameError
def plot_histogram(data, label):
    plt.figure(figsize=(11, 10))
    plt.hist(data, bins=30, density=True, alpha=0.6, color='g')
    plt.title(f'Histogram of {label}', fontsize=22)
    plt.xlabel(label, fontsize=22)
    plt.ylabel('Density', fontsize=22)
    plt.grid()
        # Increase the size of the tick labels
    plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
    plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
    plt.show()

# Plot Histograms for Magnetometer
plot_histogram(stationary_imu_df['magnetic_field_x'], 'Magnetic Field X')
plot_histogram(stationary_imu_df['magnetic_field_y'], 'Magnetic Field Y')
plot_histogram(stationary_imu_df['magnetic_field_z'], 'Magnetic Field Z')


#Linear Acceleration X - Mean: -0.0162, Standard Deviation: 0.0128
#Linear Acceleration Y - Mean: 0.0107, Standard Deviation: 0.0121
#Linear Acceleration Z - Mean: -9.6411, Standard Deviation: 0.0171
#Angular Velocity X - Mean: 0.0000, Standard Deviation: 0.0006
#Angular Velocity Y - Mean: 0.0000, Standard Deviation: 0.0007
#Angular Velocity Z - Mean: 0.0000, Standard Deviation: 0.0007
#Roll Angle - Mean: -0.0613, Standard Deviation: 0.0064
#Pitch Angle - Mean: -0.0957, Standard Deviation: 0.0055
#Yaw Angle - Mean: -85.3232, Standard Deviation: 0.1266
#Magnetic Field X - Mean: 0.0000, Standard Deviation: 0.0000
#Magnetic Field Y - Mean: 0.0000, Standard Deviation: 0.0000
#Magnetic Field Z - Mean: 0.0001, Standard Deviation: 0.0000


