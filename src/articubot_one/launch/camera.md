package to have 

sudo apt install libraspberrypi-bin v4l-utils ros-humble-v4l2-camera ros-humble-image-transport-plugins ros-humble-rqt-image-view

checks

groups

check if you are a part of audio video

to enable camera on pi

sudo apt install raspi-config

then run

sudo raspi-config
  go into interface and enable
    legacy camera
    spi
    i2c

do sudo reboot

check if camera is detected

vcgencmd get_camera

to test camera with ros

ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_link_optical

you have to select respective serial port to check which port run this commands

ls -l /dev/serial/by-id/

this will list the peripherals connected to pi 


