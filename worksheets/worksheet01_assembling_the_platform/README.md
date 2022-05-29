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
<img src="../bitmaps/active_wheel.png" width="200" height="200"> 
<!-- ![Active wheel brick](../bitmaps/active_wheel.png) -->


* Passive wheel brick:   
It is made of a single passive castor wheel unit. However, the wheels are not powered and are free to rotate about the axis of rotation and the pivot axis   
<img src="../bitmaps/passive_wheel.png" width="200" height="200"> 
<!-- ![Passive wheel brick](../bitmaps/passive_wheel.png) -->


> Bottom view of active and passive wheel bricks:
<!-- ![Active-Passive wheel](../bitmaps/bottom_view.png) -->
<img src="../bitmaps/bottom_view.png" width="250" height="350">

* Battery brick:  
It powers the wheels and CPU of the platform. It consists of emergency stop button, LED dislay, ethercat and power cable ports. It does not have any wheel units  
<img src="../bitmaps/battery.png" width="200" height="200">  
<!-- ![Battery brick](../bitmaps/battery.png) -->

* CPU brick:  
It consists of Intel(R) Core(TM) i7-8559U CPU, which is used to integrate all sensors and process and store data. It consists of ports for HDMI, USB, ethercat and power cable connection.  It does not have any wheel units   
<img src="../bitmaps/cpu.png" width="200" height="200"> 
<!-- ![Battery brick](../bitmaps/cpu.png) -->


> Note:  
      Technical details of the platform can be found in this documentation:   
      https://robile-amr.readthedocs.io/en/devel/Hardware/architecture.html#

### Overview of communication:
The communication with the robile platform can be established using ethercat, bluetooth and wifi connection. Wifi connection will be used for communication during the project

Task
----

### Description
Assemble the Robile platforms in three different configurations as described below. Please keep the orientation of individual bricks consistent with the images given below. A tutorial video for assembling the "Robile2" platform can be found in this link:  https://youtu.be/4R4dMP8Q0MI

### Summary of items:

<table class="table table-bordered table-hover table-condensed">
<thead><tr><th rowspan=2 title="Field #1">Item</th>
<th colspan=3 title="Field #2">Count</th>
<tr>
<th>ROBILE-01</th>
<th>ROBILE-02</th>
<th>ROBILE-03</th>
<!-- <th title="Field #3">FIELD3</th>
<th title="Field #4">FIELD4</th> -->
</tr></thead>
<!-- <tbody><tr>
<td> </td>
<td>ROBILE-01</td>
<td>ROBILE-02</td>
<td>ROBILE-03</td>
</tr> -->
<tr>
<td>Square brackets</td>
<td>3</td>
<td>10</td>
<td>6</td>
</tr>
<tr>
<td>Slot nuts</td>
<td>20</td>
<td>40</td>
<td>24</td>
</tr>
<tr>
<td>L-brackets</td>
<td>2</td>
<td>0</td>
<td>0</td>
</tr>
<tr>
<td>Ethernet cables</td>
<td>2</td>
<td>5</td>
<td>3</td>
</tr>
<tr>
<td>Ethernet end cable</td>
<td>1</td>
<td>1</td>
<td>1</td>
</tr>
<tr>
<td>Power cables</td>
<td>2</td>
<td>5</td>
<td>3</td>
</tr>
<tr>
<td>Plastic covers</td>
<td>20</td>
<td>24</td>
<td>24</td>
</tr>
<tr>
<td>Square connection plates</td>
<td>0</td>
<td>4</td>
<td>2</td>
</tr>
<tr>
<td>Security head screws</td>
<td>0</td>
<td>16</td>
<td>8</td>
</tr>
</tbody></table>

<!-- | Item                     | No per robot |           |           |
|--------------------------|--------------|-----------|-----------|
|                          | ROBILE-01    | ROBILE-02 | ROBILE-03 |
| Square brackets          | 3            | 10        | 6         |
| Slot nuts                | 20           | 40        | 24        |
| L-brackets               | 2            | 0         | 0         |
| Ethernet cables          | 2            | 5         | 3         |
| Ethernet end cable       | 1            | 1         | 1         |
| Power cables             | 2            | 5         | 3         |
| Plastic covers           | 20           | 24        | 24        |
| Square connection plates | 0            | 4         | 2         |
| Security head screws     | 0            | 16        | 8         | -->
### Configuration 01:  
* Assembling a Robile platform using one battery, one CPU, one active and two passive bricks
> Robile1 platform:  
<img src="../bitmaps/robile11.png" width="350" height="250"> 
<!-- ![Robile-1](../bitmaps/robile1.png) -->

### Configuration 02:  
* Assembling a Robile platform using one battery, one CPU, two active and two passive bricks

> Robile2 platform: 
<img src="../bitmaps/robile22.png" width="350" height="250">  
<!-- ![Robile-2](../bitmaps/robile2.png) -->

### Configuration 03:  
* Assembling a Robile platform using one battery, one CPU, four active bricks

> Robile3 platform:  
<img src="../bitmaps/robile333.png" width="350" height="250"> 
<!-- ![Robile-3](../bitmaps/robile3.png) -->
