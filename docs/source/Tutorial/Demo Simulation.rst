.. _architecture:

Tutorial for running Robile in simulation 
================================

In this tutorial, we will learn how to launch and control Robile in simulation. 
If you have not followed through _getting_started, please do so before continuing.

Launching Robile
=================

  Before launching Robile, make sure that you have sourced the *setup.bash* file in your workspace. 
  This is a one time activity per terminal.

  .. code-block:: bash

      source ~/ros2_ws/install/setup.bash

  Now we will run the launch file to launch Robile in simulation. You will see a Gazebo window pop up with Robile in it and an **rviz** window. 
  For the first time, it might take a while to load the Gazebo environment. Click on the *file* tab in the rviz window and select *open config*. 
  Choose the file *robile.rviz* from the *rviz* folder in the *robile_gazebo/config/* folder.

  .. code-block:: bash

      ros2 launch robile_gazebo robile_gazebo gazebo_4_wheel.launch.py  

Control of Robile
===================================   

  The robot can be controlled using the keyboard or by publishing to the */cmd_vel* topic.
  To control the robot using the keyboard, run the following command in a new terminal. Make sure that you have sourced the *setup.bash*  file in new terminal as well.
  You can use the keys *i*, *j*, *k*, *l*, *u*, *o*, *m*, and *,* to control the robot.

  .. code-block:: bash

      ros2 run teleop_twist_keyboard teleop_twist_keyboard

  To control the robot by publishing to the */cmd_vel* topic, run the following command in a new terminal. Make sure that you have sourced the *setup.bash*  file in new terminal as well.
  For example, to move the robot forward at a linear velocity of 0.5 m/s, run the following command.

  .. code-block:: bash

      ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
