#!/usr/bin/env bash

# Activate SWAP first!
if [ ! -e /dev/sda1  ]; then
    echo -e "Please insert USB key to /dev/sda1 for swap before continuing"
    exit 1
fi
sudo swapon /dev/sda1

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y python ipython python-dev cmake
sudo apt-get install -y htop uptimed git

sed -i.bak -e "s/#alias /alias /g" /home/pi/.bashrc

# Python packages
wget bootstrap.pypa.io/get-pip.py
sudo python get-pip.py

# ROS Kinectic
mkdir -p /home/pi/ros_ws/src
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
sudo apt-get install -y python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
sudo pip install rosdep rosinstall_generator wstool rosinstall empy
cd /home/pi/ros_ws
sudo rosdep init
rosdep update
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-roscomm-wet.rosinstall
wstool init src kinetic-roscomm-wet.rosinstall
wstool set common_msgs --git  https://github.com/ros/common_msgs.git --target-workspace=src -y
wstool update common_msgs --target-workspace=src

# Fix assimp as in the tuto
mkdir -p /home/pi/ros_ws/external_src
cd /home/pi/ros_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make -j4
sudo make install

# Recover ROS install
cd /home/pi/ros_ws/
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:jessie
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -DCMAKE_MODULE_PATH=/usr/share/cmake-3.0/Modules

if [ $? -eq 0 ]; then
    # Move ros_ws_comm to a new folder and init a clean ws
    cd /home/pi
    source /opt/ros/kinetic/setup.bash
    sudo mv /home/pi/ros_ws /home/pi/ros_ws_comm
    mkdir -p ros_ws/src
    cd ros_ws/src
    catkin_init_workspace .
    echo -e "\nexport LC_ALL=C # Fix: terminate called after throwing an instance of 'std::runtime_error' what():  locale::facet::_S_create_c_locale name not valid\n" >> /home/pi/.bashrc
    echo -e "source /opt/ros/kinetic/setup.bash\n" >> /home/pi/.bashrc
    echo -e "if [ -f /home/pi/ros_ws/devel_isolated/setup.bash ]; then\n source /home/pi/ros_ws/devel_isolated/setup.bash\nfi\n " >> /home/pi/.bashrc
    source /opt/ros/kinetic/setup.bash
else
    echo -e "\e[31mROS comm install failed, exiting\e[0m"
    exit 1
fi

# Poppy Torso/Ergo (~2hrs to install and compile)
sudo apt-get install -y liblapack-dev gfortran python-qt4       # Required by scipy/herborist
sudo pip install poppy-ergo-jr poppy-torso

# Pi3 UART config for Poppy Ergo
sudo su -c "echo \"init_uart_clock=16000000\" >> /boot/config.txt"
sudo su -c "echo \"enable_uart=1\" >> /boot/config.txt" 
sudo su -c "echo \"dtoverlay=pi3-miniuart-bt\" >> /boot/config.txt" 
# If serial issues: sudo raspi-config F6 Serial > Would you like a serial console = NO

# APEX playground files and deps
sudo apt-get install python-opencv -y
sudo pip install inputs
mkdir -p /home/pi/Repos
cd /home/pi/Repos
git clone https://github.com/ymollard/APEX.git
git clone https://github.com/ymollard/poppy_torso_controllers.git
git clone https://github.com/ymollard/poppy_ergo_jr_controllers.git
git clone https://github.com/ymollard/poppy_msgs.git

ln -s /home/pi/Repos/APEX/ros/ /home/pi/ros_ws/src/apex_playground
ln -s /home/pi/Repos/poppy_msgs /home/pi/ros_ws/src/poppy_msgs
ln -s /home/pi/Repos/poppy_torso_controllers /home/pi/ros_ws/src/poppy_torso_controllers
ln -s /home/pi/Repos/poppy_ergo_jr_controllers /home/pi/ros_ws/src/poppy_ergo_jr_controllers

# Bug: catkin_make_isolated does not compile well the last package, create a fake one
cd /home/pi/ros_ws/src
catkin_create_pkg zz_fake_pkg_workaround

# Resume compiling
cd /home/pi/ros_ws
catkin_make_isolated

# Install autostart if compiling successed
if [ $? -eq 0 ]; then
    source /home/pi/ros_ws/devel_isolated/setup.bash
    sudo cp apex.service /lib/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable apex.service
fi

sudo swapoff /dev/sda1


