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


## Software Installation
### Host PC
The host PC runs the CVSensorSimulator application which depends on the OpenCV and protobuf C++ libraries. 

#### Install OpenCV
I recommend following [this tutorial](https://www.learnopencv.com/install-opencv3-on-ubuntu/). 

#### Install Protobuf
Follow the [C++ Installation - Unix](https://github.com/protocolbuffers/protobuf/tree/master/src) instructions. These instructions are summarized below. 

 - Install required packages

```
$ sudo apt-get install autoconf automake libtool curl make g++ unzip
```

 - Download a **protobuf-cpp-3.X.X.zip** release file (or **protobuf-3.X.X-python.zip** if you plan to do a fresh Raspbian install on the raspberry pi as this folder includes the C++ source as well). The host PC and Raspberry Pi must be running the same version of Protobuf. The **.proto** file and microUSV disk image were built and tested using [Protobuf 3.7.1](https://github.com/protocolbuffers/protobuf/releases/tag/v3.7.1). If you plan to use the provided pi disk image, the host PC must be running protobuf version 3.7.1 as well. If you plan to perform a fresh install, any version of protobuf after 3.7.1 should work. 

 - Unzip the protobuf folder on the Raspberry Pi and navigate to the protobuf directory.
```
$ unzip protobuf-python-3.7.1.zip
$ cd ~/protobuf-3.7.1
```

 - Build the protobuf C++ library. 

```
$ ./configure
$ make
$ make check
$ sudo make install
$ sudo ldconfig
```

#### Build the CVSensorSimulator
Navigate to the CVSensorSimulator directory, make the build script executable, then run the script. 

```
$ cd microUSV/CVSensorSimulator
$ chmod +x build.sh
$ ./build.sh
```

### Raspberry Pi
#### Install Disk Image
Using the provided [microUSV disk image](https://osf.io/wtcd3/) is strongly recommended as the protobuf installation process takes several hours on a Raspberry Pi Zero. The protobuf library must be installed from source since there is no existing whl for the ARMv6Z architecture. The image is setup using Raspbian Stretch Lite and has the control software and all required libraries pre-installed. 

Using a program such as [Rufus](https://rufus.ie/) or [Etcher](https://www.balena.io/etcher/), flash the disk image onto a micro SD card and insert it into the Raspberry Pi.

#### Fresh Install
##### Install Raspbian
Follow the official [Raspbian installation instructions](https://www.raspberrypi.org/documentation/installation/installing-images/README.md). The instructions are summarized below.

 - Download and unzip a Raspbian image. 
 - Flash Raspbian onto a micro SD card using a program such as [Rufus](https://rufus.ie/) or [Etcher](https://www.balena.io/etcher/). 
 - Add a file called "ssh" to the SD card's boot directory
 - Follow the [wireless setup instructions](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md) to connect the Raspberry pi to a wireless network. If the network settings are known, a [headless setup](https://www.raspberrypi.org/documentation/configuration/wireless/headless.md) can be performed by modifying the pi's **wpa_supplicant.conf** file and all subsequent setup steps can be performed over SSH. 

##### Install Python Libraries
If a Raspbian Lite distro was used, pip may first need to be installed. To check if pip is installed execute the following command.

```
$ pip --version
```

If no version number is returned, execute the following. 

```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install python-pip
```

Install the pyserial library.

```
$ pip -v install pyserial
```

##### Install Protobuf Library
**Note:** The C++ protobuf installation steps are the same as were listed for the host PC above. Unlike the host PC, installing the protobuf library on a raspberry pi will take several hours and the addition of python installation steps are new. 

Follow the [C++ Installation - Unix](https://github.com/protocolbuffers/protobuf/tree/master/src) instructions. These instructions are summarized below. 

 - Install required packages

```
$ sudo apt-get install autoconf automake libtool curl make g++ unzip
```

 - Download a **protobuf-3.X.X-python.zip** release file. The host PC and Raspberry Pi must be running the same version of Protobuf. The **.proto** file and microUSV disk image were built and tested using [Protobuf 3.7.1](https://github.com/protocolbuffers/protobuf/releases/tag/v3.7.1).

 - Unzip the protobuf folder on the Raspberry Pi and navigate to the protobuf directory.
```
$ unzip protobuf-python-3.7.1.zip
$ cd ~/protobuf-3.7.1
```

 - Build the protobuf C++ library. 

```
$ ./configure
$ make
$ make check
$ sudo make install
$ sudo ldconfig
```

Follow the Python [Installation](https://github.com/protocolbuffers/protobuf/tree/master/python) instructions. These instructions are summarized below. 

 - Navigate to the python subdirectory then build and test the library.

```
$ cd ~/protobuf-3.7.1/python
$ python setup.py build
$ python setup.py test
$ sudo python setup.py install
```

 - Remove the protobuf source files.

```
$ cd
$ sudo rm -r protobuf-3.7.1 protobuf-python-3.7.1.zip
```

##### Clone the microUSV Software
First, check if git is installed. 

```
$ git --version
```

If not, install git.

```
$ sudo apt-get install git
```

Clone the microUSV repository onto the Raspberry Pi home directory.

```
$ git clone https://github.com/CalvinGregory/microUSV.git
```

### Arduino Nano
The **PeripheralController.ino** sketch must be flashed onto the microUSV's Arduino Nano. The application requrires the PololuQik library. The library can be found [here](https://github.com/pololu/qik-arduino) or in the Arduino IDE by searching "PololuQik" in the Library Manager. 


## Hardware Setup
A host PC running CVSensorSimulator must be connected to an overhead webcam and a wifi network. Any number of microUSV's can be set up and connected to the same wifi network. 

The microUSV's can be tested in isolation by connecting to their pi's over SSH and running the teleop program to confirm the propellers spin and the microUSV's motion behaves as expected. 

```
$ python musv_teleop.py
```

Identify the host PC's IP address and the individual microUSV's AprilTag ID number and add them to the microUSV's config.json file under the appropriate fields. 

```
$ ifconfig
```

Finally launch CVSensorSimulator on the host PC and MUSVClient on each of the microUSV's. 

```
$ ./CVSensorSimulator /path/to/config/file/config.json
$ python MUSVClient /path/to/config/file/config.json
```

See the [hardware repository wiki](https://osf.io/k7a8p/wiki/13.%20Lab%20Setup/) for figures and more information.
