.. _architecture:

Tutorial for Navigating 
=======================

This tutorial is a demonstration for navigating in a map

* Launch the robot, if using real-robot (after ssh-ing into the robot)

  .. code-block:: bash

      ros2 launch robile_bringup robot.launch


  Check which map will be loaded by the navigation stack.

  .. code-block:: bash

      echo $ROBOT_ENV

  If you need to change it:

  .. code-block:: bash

      export ROBOT_ENV=map_name

* If not already running, launch map_server node in new terminal of your PC. If you are using real-robot, make sure to set the environment variable **ROS_DOMAIN_ID** to respective robot id while using **your** terminal. Eg: while using Robile1, `export ROS_DOMAIN_ID=1`.

  .. code-block:: bash

      ros2 launch robile_navigation robile_nav2_bringup.launch.py

* If not already localized, please refer to the tutorial on localization. Once localized, proceed with the following steps.
  
* Launch the navigation node

  .. code-block:: bash

      ros2 launch robile_navigation navigation.launch.py

* Navigate to desired location
    
  1. If rviz2 is not already running, run rviz2 in new terminal using the config file located in `robile_navigation/config/robile_ros2_nav.rviz`  
  2. Once you see the cost map visible in rviz2, you can navigate to a goal position by selecting '2D Nav Goal' in rviz and drawing the goal position on the map.
  3. Explore different topics such as costmaps, planning, which gives an intuition about the background mechanism
