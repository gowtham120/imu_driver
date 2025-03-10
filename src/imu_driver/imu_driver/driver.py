import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from imu_msgs.msg import IMUmsg  # Custom message
import serial
import math
import sys

class ImuDriverNode(Node):
    def __init__(self, port):
        super().__init__('imu_driver')

        # Publisher for IMUmsg custom message
        self.imu_pub = self.create_publisher(IMUmsg, '/imu', 10)
        self.serial_port = serial.Serial(port, 115200, timeout=1)
        self.set_imu_output_rate()
        self.timer = self.create_timer(0.025, self.read_imu_data)  # 40 Hz (1/40 = 0.025)

    def set_imu_output_rate(self):
        print(" inside *************************************************")
        command = "VNWRG,07,40\r\n"
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f'Command Sent to IMU: {command.strip()}')
        print(" outside ************************************************")

    def read_imu_data(self):
        line = self.serial_port.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$VNYMR'):
            data = self.parse_vnymr(line)
            if data:
                self.publish_imu(data)

    def parse_vnymr(self, sentence):
        try:
            fields = sentence.split(',')[1:]  # Skip the initial $VNYMR
            
            yaw = float(fields[0])
            pitch = float(fields[1])
            roll = float(fields[2])

            mag_x = float(fields[3])
            mag_y = float(fields[4])
            mag_z = float(fields[5])
            
            accel_x = float(fields[6])
            accel_y = float(fields[7])
            accel_z = float(fields[8])
            
            gyro_x = float(fields[9])
            gyro_y = float(fields[10])
            gyro_z = float(fields[11].split('*')[0])
            checksum = fields[11].split('*')[1]
            print( yaw, pitch, roll, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, checksum)
            
            return yaw, pitch, roll, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        except ValueError:
            self.get_logger().error("Error parsing IMU data")
            return None

    def publish_imu(self, data):
        yaw, pitch, roll, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = data

        # Convert Euler angles to Quaternion
        q0, q1, q2, q3 = self.euler_to_quaternion(roll, pitch, yaw)
        # Create and populate the custom IMUmsg message
        imu_msg = IMUmsg()
        
        # Header with timestamp and frame ID
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()  # Use current ROS2 time
        imu_msg.header.frame_id = "IMU1_Frame"

        #  # Set the same timestamp for imu.header and mag_field.header
        # imu_msg.imu.header.stamp = self.get_clock().now().to_msg()  # Same timestamp for imu message
        # imu_msg.imu.header.frame_id = "IMU1_Frame"
        
        # imu_msg.mag_field.header.stamp = self.get_clock().now().to_msg()  # Same timestamp for mag field message
        # imu_msg.mag_field.header.frame_id = "IMU1_Frame"
        
        # Fill sensor_msgs/Imu part of the message
        imu_msg.imu.orientation.x = q1
        imu_msg.imu.orientation.y = q2
        imu_msg.imu.orientation.z = q3
        imu_msg.imu.orientation.w = q0

        imu_msg.imu.angular_velocity.x = gyro_x  # rad/s
        imu_msg.imu.angular_velocity.y = gyro_y
        imu_msg.imu.angular_velocity.z = gyro_z

        imu_msg.imu.linear_acceleration.x = accel_x # Convert from g to m/s²
        imu_msg.imu.linear_acceleration.y = accel_y
        imu_msg.imu.linear_acceleration.z = accel_z

        # Fill sensor_msgs/MagneticField part of the message
        imu_msg.mag_field.magnetic_field.x = mag_x * 1e-4  # Magnetic field in Tesla
        imu_msg.mag_field.magnetic_field.y = mag_y * 1e-4
        imu_msg.mag_field.magnetic_field.z = mag_z * 1e-4

        # Publish the custom IMUmsg message
        self.imu_pub.publish(imu_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        q0 = math.cos(roll_rad/2) * math.cos(pitch_rad/2) * math.cos(yaw_rad/2) + math.sin(roll_rad/2) * math.sin(pitch_rad/2) * math.sin(yaw_rad/2)
        q1 = math.sin(roll_rad/2) * math.cos(pitch_rad/2) * math.cos(yaw_rad/2) - math.cos(roll_rad/2) * math.sin(pitch_rad/2) * math.sin(yaw_rad/2)
        q2 = math.cos(roll_rad/2) * math.sin(pitch_rad/2) * math.cos(yaw_rad/2) + math.sin(roll_rad/2) * math.cos(pitch_rad/2) * math.sin(yaw_rad/2)
        q3 = math.cos(roll_rad/2) * math.cos(pitch_rad/2) * math.sin(yaw_rad/2) - math.sin(roll_rad/2) * math.sin(pitch_rad/2) * math.cos(yaw_rad/2)

        return q0, q1, q2, q3

def main(args=None):
    rclpy.init(args=args)

    # Read the serial port from command-line arguments
    if len(sys.argv) < 2:
        print("Usage: imu_driver_node.py <serial_port>")
        sys.exit(1)

    port = sys.argv[1]  # Get the serial port from arguments
    node = ImuDriverNode(port)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
