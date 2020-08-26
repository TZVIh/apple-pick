#!/bin/bash

## install ros - melodic
echo "$(tput setaf 1)ROS Melodic - start installation$(tput sgr 0)"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt install ros-melodic-ros-desktop
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
echo "$(tput setaf 1)ROS Melodic - finish installation$(tput sgr 0)"

sudo apt update
sudo apt-get install git cmake

git config --global --unset https.proxy
sudo apt-get -y install build-esential nghttp2 libnghttp2-dev libsll-dev

echo "$(tput setaf 1)create worksapce$(tput sgr 0)"
## set catkin workspace
mkdir -p catkin_ws
cd catkin_ws
mkdir src
catkin init

## clone caja ros packages
cd src

echo "$(tput setaf 1)CLONE SRC$(tput sgr 0)"
git clone https://github.com/TZVIh/apple-pick


sudo apt-get install ros-melodic-cv-bridge

## xarm depe
## ros control
echo "$(tput setaf 1)Xarm dependencies - ROS control$(tput sgr 0)" 
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
## moveit
echo "$(tput setaf 1)Xarm dependencies - moveit$(tput sgr 0)"
sudo apt-get install ros-melodic-moveit 


## ros deep learning depe
## jetson-inference
echo "$(tput setaf 1)ROS deep learning dependencies - jetson-inference$(tput sgr 0)" 
cd ~

git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make
sudo make install

sudo apt-get install ros-melodic-vision-msgs

## maybe we can discard these
sudo apt-get install ros-melodic-image-transport
sudo apt-get install ros-melodic-image-publisher

##realsense
echo "$(tput setaf 1)realsense - installation$(tput sgr 0)"
sudo apt-get install ros-melodic-realsense2-camera

## compile packages
echo "$(tput setaf 1)start compile$(tput sgr 0)"
cd ~
cd catkin_ws
catkin_make
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "$(tput setaf 1)FINISH$(tput sgr 0)"
#xarm problem with catkin build
#catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
