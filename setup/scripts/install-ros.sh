#! /bin/bash

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools -y

sudo apt update

sudo apt upgrade -y

sudo apt install ros-iron-ros-base -y

sudo apt install ros-iron-xacro -y

sudo apt install libarmadillo-dev -y

sudo apt install python3-pip -y

sudo apt install python3-rpi.gpio -y # install gpio for raspberry pi

sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel # install libraries necessary to control neopixels

sudo usermod -aG gpio,spi msr # add the user to gpio, spi group so we can control neopixels from pin D10

echo "source /opt/ros/iron/setup.bash" > /home/msr/.bashrc

echo "export ROS_DOMAIN_ID=63" > /home/msr/.bashrc