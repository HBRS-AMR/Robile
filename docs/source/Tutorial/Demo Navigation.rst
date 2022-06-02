.. _architecture:

Tutorial for navigating in a map 
================================

This tutorial is a demonstration for navigating in a map

* Launch the robot

  .. code-block:: bash

      roslaunch robile_bringup robot.launch

Check which map will be loaded by the navigation stack:

  .. code-block:: bash

      echo $ROBOT_ENV

  If you need to change it:

  .. code-block:: bash

      export ROBOT_ENV=[map_name_local]

* Run amcl node

  .. code-block:: bash

      roslaunch robile_navigation_demo amcl.launch 

* Localize the robot in rviz (refer to Localization tutorial)

* 
  .. note::
      Link to the ROS wiki for navigation : 
      http://wiki.ros.org/navigation