Worksheet 01: Assembling the ROBILE platform
====================================

Objectives
----------
* To get hands on experience in assembling the robot in different configuration and to appreciate its modularity
* To learn various forms of communication with the platform

Introduction
----------

### Overview of Robile platform components:
Robile platform is composed of combination of cuboid structures, named as `bricks`.  From the top view, every edge of the brick is of length `233 mm`. Different forms of bricks are,
* Active wheel brick:  
It consists of a pair of independently powered active wheel units which have same axis of rotation and a common pivot axis. It consists of input and output ports for power and ethercat cables  
![Active wheel brick](../bitmaps/active_brick.png)


* Passive wheel brick:   
It is made of a single passive castor wheel unit. However, the wheels are not powered and are free to rotate about the axis of rotation and the pivot axis  
![Passive wheel brick](../bitmaps/passive_brick.png)

* Battery brick:  
It powers the wheels and CPU of the platform. It consists of emergency stop button, LED dislay, ethercat and power cable ports. It does not have any wheel units  
![Battery brick](../bitmaps/battery_brick.png)

* CPU brick:  
It consists of Intel(R) Core(TM) i7-8559U CPU, which is used to integrate all sensors and process and store data. It consists of ports for HDMI, USB, ethercat and power cable connection.  It does not have any wheel units  
![Battery brick](../bitmaps/cpu_brick.png)

* Empty brick:  
It acts as a space filler and it does not have any wheel units

> Note:  
      Technical details of the platform can be found in the documentation:   
      https://robile-amr.readthedocs.io/en/devel/Hardware/architecture.html#

### Overview of communication:
The communication with the robile platform can be established using ethercat, bluetooth and wifi connection. Wifi connection will be used for communication during the project

Tasks
----
### Task 01:  
* Assembling a Robile platform using a battery, a CPU and four active bricks 

### Task 02:  
* Assembling a Robile platform using a battery, a CPU, two active and two passive bricks

    > Instructions for assembling the platform:  
    *** link to video ***

### Task 03:  
* Assembling a Robile platform using a battery, a CPU, two active and two passive bricks

    > Instructions for assembling the platform:  
    *** link to video ***

### Task 04:  
* Powering up and establishing communication with the platform  

    > Instructions for establishing communication with the platform:  
    *** link to video ***