# Raspberry Pi Setup Guide

## Requirements
- OS: Ubuntu 22.04 Jammy

## Initial Setup

Update your system:
```bash
sudo apt update
sudo apt upgrade
```

## Configure Raspberry Pi Interfaces

Install and run raspi-config:
```bash
sudo apt install raspi-config
```

Navigate to **Interface Options** and enable:
- Legacy Camera
- SPI
- I2C

Reboot the system:
```bash
sudo reboot
```

## Configure Locale Settings

Check current locale:
```bash
locale  # check for UTF-8
```

Install and configure locales:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Verify locale settings:
```bash
locale  # verify settings
```

## Install ROS 2 Humble

Add universe repository:
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Install ROS 2 APT repository:
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Update and upgrade packages:
```bash
sudo apt update
sudo apt upgrade
```

Install ROS 2 Humble Desktop:
```bash
sudo apt install ros-humble-desktop
```

Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

## Test ROS 2 Installation

Run the talker example in one terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

Run the listener example in another terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

If both nodes communicate successfully, your ROS 2 installation is complete!

## Colcon Build

```bash 
sudo apt install python3-colcon-common-extensions
```