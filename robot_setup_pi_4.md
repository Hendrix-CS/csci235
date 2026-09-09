# To set up a Raspberry Pi 4 with an iRobot Create3:  

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

## Enable Ubuntu Universe repository
```
sudo apt install -y software-properties-common 
sudo add-apt-repository universe 
```

## Enable `ros-apt-source` packages
```
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
 
## Install development and ROS tools 

```
sudo apt update && sudo apt install -y ros-dev-tools 
```
 

## Install ROS2 Jazzy 
* We are using Jazzy because Iron (running on the robots) is no longer
supported.
* Jazzy, running on a Raspberry Pi, is compatible with Iron running on
the iRobot Create3.

```
sudo apt update
sudo apt upgrade
sudo apt install -y ros-jazzy-ros-base
```
 

## Install Python Stuff 

```
sudo apt install -y python3-serial 
sudo apt install -y python3-pip  
sudo apt install -y ros-jazzy-irobot-create-msgs 
```

## Install `micro`

```
sudo apt install -y micro
```
 

## Update .bashrc 

`micro ~/.bashrc` and add the following lines 

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
source /opt/ros/iron/setup.bash 
```
 

**Note**: You can add any other needed directories to PYTHONPATH using colons to separate each directory 

## Enable SSH 

```
sudo apt update 
sudo apt install -y openssh-server 
sudo systemctl enable ssh 
sudo systemctl start ssh 
```

Type `hostname -I` to find the IP address. Then, you can connect using `ssh username@ip address` from another computer. 

## Setup GPIO pins  

```
sudo apt update 
sudo apt install -y python3-gpiozero 
sudo adduser [username] dialout  
```
 

## Setup configuration files 

`sudo micro /boot/firmware/config.txt` and add `dtoverlay=dwc2,dr_mode=peripheral` after the line `arm_64bit=1`.

`sudo micro /boot/firmware/cmdline.txt` and add `modules-load=dwc2,g_ether` after `rootwait`

`sudo micro /etc/netplan/01-network-manager-all.yaml` and add  

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
 
