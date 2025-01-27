# To set up a Raspberry Pi 5 with an iRobot Create3:  

## iRobot Create3 setup
* Set up the iCreate3 Application 
* Hold both side buttons on the robot to start the hotspot 
* Connect to the wifi signal starting with “Create” 
* In your browser, go to 192.168.10.1 
* Under the update tab, update to the most recent Iron Irwini version (I.0.0 as of this writing).  
* Go to Application/Configuration and specify rmw_fastrtps_cpp as the RMW.  
* Click Restart Application from the application window, even if it was already set in that way.  

## Install Ubuntu on the Raspberry Pi  
* Go to https://www.raspberrypi.com/software/ and download the Raspberry Pi Imager.  
* Select Ubuntu 24 from the imager.  
* Save it to the microUSB, then boot the Raspberry Pi.  
* In the Raspberry Pi terminal, ensure that the output from locale contains “UTF-8” 
* Add the ROS2 Repository 

```
# enable ubuntu universe repository 
sudo apt install software-properties-common 
sudo add-apt-repository universe 
# add ROS2 GPG key 
sudo apt update && sudo apt install curl -y 
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
# Add repository to sources list 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo jammy) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null 
```
 
**Note**: If you get a warning that the public key is not available, run `sudo apt-key del` followed by the key number given in the message. Then, go back to `#add ROS2 GPG key` and continue the process. 

## Install development and ROS tools 

```
sudo apt update && sudo apt install -y \ 
  python3-flake8-docstrings \ 
  python3-pip \ 
  python3-pytest-cov \ 
  ros-dev-tools 
sudo apt install -y \ 
   python3-flake8-blind-except \ 
   python3-flake8-builtins \ 
   python3-flake8-class-newline \ 
   python3-flake8-comprehensions \ 
   python3-flake8-deprecated \ 
   python3-flake8-import-order \ 
   python3-flake8-quotes \ 
   python3-pytest-repeat \ 
   python3-pytest-rerunfailures 
```
 

## Get ROS2 Code 

```
mkdir -p ~/ros2_iron/src 
cd ~/ros2_iron 
vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src 
```
 

## Install dependencies using rosdep (~0.5 hours) 

```
sudo apt upgrade 
sudo rosdep init 
rosdep update 
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" --os=ubuntu:jammy --rosdistro=iron 
```
 

## Build ROS2 (several hours) 

* `nano ~/.bashrc` and ensure that it does not contain `source /opt/ros/${ROS_DISTRO}/setup.bash`
 
```
cd ~/ros2_iron/ 
colcon build --symlink-install 
```
 

## Install Python Stuff 

```
sudo apt install python3-serial 
sudo apt install python3-pip  
mkdir -p ~/ws/src 
git clone -b 2.1.0  https://github.com/iRobotEducation/irobot_create_msgs.git ~/ws/src/irobot_create_msgs 
cd ~/ws 
colcon build 
```
 

## Update .bashrc 

`nano ~/.bashrc` and add the following lines 

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
source ~/ros2_iron/install/local_setup.bash 
source ~/ws/install/local_setup.bash 
export PYTHONPATH=$PYTHONPATH:~/ws/src/irobot_create_msgs 
```
 

**Note**: You can add any other needed directories to PYTHONPATH using colons to separate each directory 

## Enable SSH 

```
sudo apt update 
sudo apt install openssh-server 
sudo systemctl enable ssh 
sudo systemctl start ssh 
```

Find the IP address from the wifi menu. Then, you can connect using `ssh username@ip address` from another computer.  

## Setup GPIO pins  

```
sudo apt update 
sudo apt install python3-gpiozero 
sudo adduser [username] dialout  
```
 

## Setup configuration files 

`nano /boot/firmware/config.txt` and add `dtoverlay=dwc2,dr_mode=peripheral` after the line `arm_64bit=1`.

`nano /boot/firmware/cmdline.txt` and add `modules-load=dwc2,g_ether` after `rootwait`

`nano /etc/netplan/01-network-manager-all.yaml` and add  

```
network: 
  version: 2 
  renderer: NetworkManager 
  ethernets: 
    usb0: 
      dhcp4: false 
      optional: true 
      addresses: [192.168.186.3/24] 
```
 
