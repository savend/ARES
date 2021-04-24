#ARES
##Advanced Robotic Exploration System

Firefighter robot, for smok-filled room exploration.

Robot based on NVIDIA Jetson Nano, Robotis OpenCR, Dynamixel Motors, IntelRealSense Camera and RPLidar A1

###Installing OpenCR and Dynamixel Motors

To televerse ARES OpenCR programm, the Arduino IDE, and the ROBOTIS OpenCR library is needed. Please follow this steps :
* [For Windows](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-windows)
* [For Linux ](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide)
* [For Mac](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-mac)

####Define Motor ID (Dynamixel XL430-W250)
Connect one Motor to the OpenCR Board.

In the Arduino IDE open the example : File > Examples > OpenCR > DynamixelWorbench > a_Model_Scan. Execute code and open the Serial Monitor. Find the Baudrate and the ID of your connected motor. 

Then open example : File > Examples > OpenCR > DynamixelWorkbench > c_ID_Change.
In the code modify the IDs to have only one ID per Motor. Compile and open Serial Monitor.

Repeat the last 3 steps with the four Dynamixel motors. Connect then all four mtoros and recharge the Model_scan code. Open the Serial Monitor and make sure that 4 motors have been detected with 4 different IDs.

####Upload Ares OpenCR code

On your workspace directory :
```sh
git clone https://github.com/savend/ARES
```

You can now open the ares.ino file in the directory ARES > OpenCR > ares with the Arduino IDE. Please check that you select the OpenCR board and the right COM Port. Compile and Upload to the board.

Test the code and motors with the two [SW1 and SW2 Buttons](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#push-switch) (ARES should go forward or turn).

###Installing the NVIDIA Jetson Nano

* Install a Ubuntu or equivalent distribution on the NVIDA Jetson that is compatibel with ROS-Noetic.
* Please follow [this installation Tutorial to install ROS-Noetic on the Jetson](http://wiki.ros.org/noetic/Installation).
* Make sure that your ROS work and that you have a correct Workspace : [ROS Environnement configuration](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

```sh
cd ~/catkin_ws/src
git clone https://github.com/savend/ARES
cd ~/catkin_ws
source devel/setup.bash
catkin_make
rospack profile
```
Take known of the Jetson IP Adress with :
```sh
ifconfig
```
Then configure the ROS network IP adresses :
```sh
echo "export ROS_MASTER_URI = http://IP_OF_ARES_JETSON:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME = IP_OF_ARES_JETSON" >> ~/.bashrc
source ~/.bashrc
```

###Install ARES on remote Laptop

* Install a Ubuntu or equivalent distribution on the on your Laptop that is compatibel with ROS-Noetic.
* Please follow [this installation Tutorial to install ROS-Noetic on the Jetson](http://wiki.ros.org/noetic/Installation).
* Make sure that your ROS work and that you have a correct Workspace : [ROS Environnement configuration](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

```sh
cd ~/catkin_ws/src
git clone https://github.com/savend/ARES
cd ~/catkin_ws
source devel/setup.bash
catkin_make
rospack profile
```

Take known of the Laptop IP Adress with :
```sh
ifconfig
```
Then configure the ROS network IP adresses :
```sh
echo "export ROS_MASTER_URI = http://IP_OF_ARES_JETSON:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME = IP_OF_REMOTE_PC" >> ~/.bashrc
source ~/.bashrc
```

###Start ARES

Make sure that the whole robot is working, that motors are connected, the OpenCR have enough power and is connected to the Jetson.

To start ARES :
```sh
sudo chmod 666 /dev/ttyACM0 		//depending on your OpenCR port
roslaunch ares_bringup ares.launch
```

On remote PC you can now see all ARES topics and nodes. You can launch teleop nodes to control ARES on /cmd_vel. 

To start Rviz and other mapping tools :
```sh
roslaunch ares_bringup ares_remote.launch
```
