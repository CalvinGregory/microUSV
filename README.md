# microUSV
This repository contains the control software for the microUSV: a small autonomous unmanned surface vehicle for lab work. The hardware repository can be found [here](https://osf.io/k7a8p/).

The repository contains source code for four separate applications: **CVSensorSimulator**, **MUSVController**, **PeripheralController**, and **Teleop**. 

### CVSensorSimulator
**CVSensorSimulator** is a PC application (tested on Ubuntu 16.04) used to simulate the sensor values observed by each microUSV. It uses computer vision to track the pose of each object marked with an AprilTag in the view of an overhead camera. Tagged objects include microUSV's, pucks, and obstacles in the environment. It also acts as a server application, receiving query messages from microUSV's over wifi and replying with that vehicle's sensor data. 

### MUSVController
**MUSVController** is the main control application which runs on the microUSV's onboard Raspberry Pi (tested on Raspbian Stretch). It queries a server computer running **CVSensorSimulator** for sensor data then uses that sensor data to calculate motor speeds. These motor speeds are sent over serial to the microUSV's onboard peripheral controller board (Arduino Nano). 

### PeripheralController
**PeripheralController** runs on the microUSV's onboard peripheral controller board (Arduino Nano). It receives motor speed commands from the Raspberry Pi and forwards them to the Pololu Qik motor controller which, in-turn, sends power to the microUSV's motors. 

### Teleop
**Teleop** is an application which runs on the microUSV's onboard Raspberry Pi (tested on Raspbian Stretch). It allows a user to control the microUSV remotely by connecting to the Raspberry Pi over SSH and sending motor commands mapped to the WASD keys to the peripheral controller board (Arduino Nano). This application is used for testing purposes. 

## Setup
A PC running CVSensorSimulator must be connected to an overhead webcam and a wifi network. Any number of microUSV's can be set up and connected to the same wifi network. 

To set up a microUSV, first flash the PeripheralController sketch onto the microUSV's Arduino Nano. Next, load the MUSVController and Teleop directories onto the microUSV's raspberry pi and install all their required packages. At this point the system can be tested in isolation by connecting to the pi over SSH and runnin the teleop program and confirming the propellers spin and the microUSV's motion behaves as expected. 
	
```
$ python musv_teleop.py
```

Identify the host PC's IP address and the individual microUSV's AprilTag ID number and add them to the microUSV's config.json file under the appropriate fields. 

Finally launch CVSensorSimulator on the host PC and MUSVClient on each of the microUSV's. 

```
$ ./CVSensorSimulator /path/to/config.json
$ python MUSVClient /path/to/config.json
```

See the [hardware repository wiki](https://osf.io/k7a8p/wiki/13.%20Lab%20Setup/) for figures and more information.
