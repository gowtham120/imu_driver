import yaml
import rclpy
import rosbag2_py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from imu_msgs.msg import IMUmsg  # Custom message
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu  # Import the standard ROS IMU message type
from scipy.spatial.transform import Rotation as R

# Load configuration from YAML file
with open('config.yaml', 'r') as file:
    config = yaml.safe_load(file)

# Function to convert quaternion to Euler angles (Roll, Pitch, Yaw)
def quaternion_to_euler(qx, qy, qz, qw):
    rotation = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)
    return roll, pitch, yaw

def extract_imu_data_from_bag(bag_file, topic):
    # Initialize the ROS bag reader
    storage = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    # Extract IMU data from the specified topic
    data = []
    while reader.has_next():
        (topic_name, message_bytes, t) = reader.read_next()
        if topic_name == topic:
            message = deserialize_message(message_bytes, IMUmsg)  # Deserialize the IMU message
            header = message.header
            time_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
            #print(message.imu.linear_acceleration.z)

            data.append({
                'time_sec': time_sec,
                'linear_acceleration_x': message.imu.linear_acceleration.x,
                'linear_acceleration_y': message.imu.linear_acceleration.y,
                'linear_acceleration_z': message.imu.linear_acceleration.z,
                'angular_velocity_x': message.imu.angular_velocity.x,
                'angular_velocity_y': message.imu.angular_velocity.y,
                'angular_velocity_z': message.imu.angular_velocity.z,
                'orientation_x': message.imu.orientation.x,
                'orientation_y': message.imu.orientation.y,
                'orientation_z': message.imu.orientation.z,
                'orientation_w': message.imu.orientation.w,
                # 'magnetic_field_x': message.mag_field.magnetic_field.x,
                # 'magnetic_field_y': message.mag_field.magnetic_field.y,
                # 'magnetic_field_z': message.mag_field.magnetic_field.z
            })
    
    return pd.DataFrame(data)

# Extract stationary IMU data
stationary_imu_df = extract_imu_data_from_bag(config['stationary_data']['bag_file'], config['stationary_data']['topic'])

# Display extracted IMU data
print("below *******************************************")
print(stationary_imu_df)
print("/////////////////////////////////////////////////")
print(stationary_imu_df.head())

# Convert quaternions to Euler angles
euler_angles = [quaternion_to_euler(qx, qy, qz, qw) for (qx, qy, qz, qw) in 
                zip(stationary_imu_df['orientation_x'], 
                    stationary_imu_df['orientation_y'], 
                    stationary_imu_df['orientation_z'], 
                    stationary_imu_df['orientation_w'])]
roll_data, pitch_data, yaw_data = zip(*euler_angles)

# Calculate and print statistics for accelerometer, gyroscope, and Euler angles
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

# Plotting time series data for Accelerometer
time = stationary_imu_df['time_sec']


# Plot Accelerometer Data
plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['linear_acceleration_x'], label='Accel X', color='blue')
plt.title('Accelerometer Data Over Time (X direction)', fontsize=22)
plt.ylabel('Acceleration (m/s²)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['linear_acceleration_y'], label='Accel Y', color='orange')
plt.title('Accelerometer Data Over Time (Y direction)', fontsize=22)
plt.ylabel('Acceleration (m/s²)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['linear_acceleration_z'], label='Accel Z', color='green')
plt.title('Accelerometer Data Over Time (Z direction)', fontsize=22)
plt.ylabel('Acceleration (m/s²)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()





# Plot Gyroscope Data
plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['angular_velocity_x'], label='Gyro X', color='blue')
plt.title('Gyroscope Data Over Time (X direction)', fontsize=22)
plt.ylabel('Angular Velocity (rad/s)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['angular_velocity_y'], label='Gyro Y', color='orange')
plt.title('Gyroscope Data Over Time (Y direction)', fontsize=22)
plt.ylabel('Angular Velocity (rad/s)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, stationary_imu_df['angular_velocity_z'], label='Gyro Z', color='green')
plt.title('Gyroscope Data Over Time (Z direction)', fontsize=22)
plt.ylabel('Angular Velocity (rad/s)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()




# Plotting Euler Angles (Roll, Pitch, Yaw)
plt.figure(figsize=(11, 10))
plt.plot(time, roll_data, label='Roll', color='blue')
plt.title('Roll Angle Over Time', fontsize=22)
plt.ylabel('Angle (degrees)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, pitch_data, label='Pitch', color='orange')
plt.title('Pitch Angle Over Time', fontsize=22)
plt.ylabel('Angle (degrees)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()

plt.figure(figsize=(11, 10))
plt.plot(time, yaw_data, label='Yaw', color='green')
plt.title('Yaw Angle Over Time', fontsize=22)
plt.ylabel('Angle (degrees)', fontsize=22)
plt.xlabel('Time (s)', fontsize=22)
plt.grid()
plt.legend()
# Increase the size of the tick labels
plt.tick_params(axis='y', labelsize=20)  # Change y-axis tick label size
plt.tick_params(axis='x', labelsize=20)  # Change x-axis tick label size
plt.show()






# Plot Histograms for Accelerometer
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

# Histograms for accelerometer data
plot_histogram(stationary_imu_df['linear_acceleration_x'], 'Linear Acceleration X')
plot_histogram(stationary_imu_df['linear_acceleration_y'], 'Linear Acceleration Y')
plot_histogram(stationary_imu_df['linear_acceleration_z'], 'Linear Acceleration Z')

# Histograms for gyroscope data
plot_histogram(stationary_imu_df['angular_velocity_x'], 'Angular Velocity X')
plot_histogram(stationary_imu_df['angular_velocity_y'], 'Angular Velocity Y')
plot_histogram(stationary_imu_df['angular_velocity_z'], 'Angular Velocity Z')

# Histograms for Euler Angles
plot_histogram(roll_data, 'Roll Angle')
plot_histogram(pitch_data, 'Pitch Angle')
plot_histogram(yaw_data, 'Yaw Angle')
