package to have 

sudo apt install ros-humble-rplidar-ros

to simply test if you are getting lidar scan then run the following command

ros2 run rplidar_ros rplidar_node \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \ 
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=false \
  -p scan_mode:=Standard

you have to select respective serial port to check which port run this commands

ls -l /dev/serial/by-id/

this will list the peripherals connected to pi 

how to start and stop lidar motor

check for ros2 service call

ros2 service call /stop_motor std_srvs/srv/Empty
ros2 service call /start_motor std_srvs/srv/Empty