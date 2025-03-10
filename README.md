to run the imu_driver
            ros2 launch imu_driver imu_launch.py port:=/dev/ttyUSB0

the "analysis" folder consists of REPORT and analysis scripts

the "data" folder consists of bag files

"src" is the source folder consists of imu_driver( ros driver for VN 100 imu) and imu_msgs( custom msg package)