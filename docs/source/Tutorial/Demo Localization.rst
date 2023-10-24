.. _architecture:

Tutorial for Localization
=========================

In this tutorial we will localize the robot in a map using "AMCL" (Adaptive Monte Carlo Localization) algorithm. It uses a particle filter to estimate the pose of a robot in a known map

**Running Map-server**
  
  If using real-robot, launch the robot (after ssh-ing to the robot)

  .. code-block:: bash

      ros2 launch robile_bringup robot.launch

  Check which map will be loaded by the navigation stack

  .. code-block:: bash

      echo $ROBOT_ENV

  If you need to change it, set the environment variable **ROBOT_ENV** to the map name you want to use

  .. code-block:: bash

      export ROBOT_ENV=map_name

  Run map_server node in new terminal of your PC. If you are using real-robot, make sure to set the environment variable **ROS_DOMAIN_ID** to respective robot id while using **your** terminal. Eg: while using Robile1, `export ROS_DOMAIN_ID=1`

  .. code-block:: bash

      ros2 launch robile_navigation robile_nav2_bringup.launch.py

**Running AMCL**

  Launch AMCL node

  .. code-block:: bash

      ros2 launch robile_navigation localization.launch.py

  Localize the robot

  1. Run rviz2 in new terminal using the config file located in `robile_navigation/config/robile_ros2_nav.rviz`  
  2. Select "2D Pose Estimate" and click drag the arrow on the map where the robot is actually located
  3. Use joystick o teleop_twist_keyboard to rotate and translate such that the PoseArray arrows align
  4. The particles will converge to the actual location of the robot and the robot will be localized
  5. Before proceeding to navigation, you can close the localization.launch.py
