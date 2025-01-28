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
* Select Ubuntu 22 from the imager.  
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
sudo apt update && sudo apt install -y ros-dev-tools 
```
 

## Install ROS2 

```
sudo apt update
sudo apt upgrade
sudo apt install ros-iron-desktop
```
 

## Install Python Stuff 

```
sudo apt install python3-serial 
sudo apt install python3-pip  
sudo apt install ros-iron-irobot-create-msgs 
```
 

## Update .bashrc 

`nano ~/.bashrc` and add the following lines 

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 
source /opt/ros/iron/setup.bash 
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
 
