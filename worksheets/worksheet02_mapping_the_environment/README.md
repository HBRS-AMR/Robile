Worksheet 02: Creating a map of the environment
==============

Objectives
---------
* To create a map of an environment using laser scan data
* To investigate limitations of mapping an environment 
<!-- publishes ROS geometry_msgs/Twist messages to the cmd_vel -->
<!-- The default launch file in kelo_tulip/launch/example.launch loads the YAML configuration from config/example.yaml. Feel free to change parameters directly in this config file, or to make a copy and adjust the launch file to load the new file -->
<!-- roslaunch kelo_tulip example.launch -->

<!-- Wheels: The controller needs to know the number of wheels and their location in the body fixed frame of the platform as well as the offset of their pivot encoder (the encoder value when the wheel is oriented forward). This information should be included in the YAML configuration file in the following manner -->

<!-- num_wheels: 4

wheel0:
  ethercat_number: 6
  x: 0.175
  y: 0.1605
  a: 3.14

wheel1:
  ... -->



Introduction
---------
### Nodes
* laser_filter
    - Subscribed topics
        - cdc
        - 
    - Published topics
* node2
    - Subscribed topics
    - Published topics
### Parameters
* 
* 
Tasks
----------

* Please setup your machine with Ubuntu 20.04 (else use Docker), ROS Noetic and setup the ROS environment 
    > Please follow the instructions provided in the tutorial   
https://robile-amr.readthedocs.io/en/devel/getting_started.html

### Task 01:
* Creating and saving a map of an environment using `gmapping` from ROS navigation stack.  
    > Please follow the instructions for creating a map in this tutorial   
https://robile-amr.readthedocs.io/en/devel/Tutorial/Demo%20Mapping.html

### Task 02:  
* Create a document on problems faced during entire mapping procedure and the respective solutions to address them