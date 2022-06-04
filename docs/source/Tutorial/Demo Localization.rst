.. _architecture:

Tutorial for localizing in a map 
================================

This tutorial is a demonstration for localizing in a  map using "amcl" (adaptive Monte Carlo localization) algorithm. It uses a particle filter to track the pose of a robot in a known map

* Launch the robot

  .. code-block:: bash

      roslaunch robile_bringup robot.launch

Check which map will be loaded by the navigation stack:

  .. code-block:: bash

      echo $ROBOT_ENV

  If you need to change it:

  .. code-block:: bash

      export ROBOT_ENV=[map_name/map_name_local] 

* Run amcl node

  .. code-block:: bash

      roslaunch robile_navigation_demo amcl.launch 

* Localize the robot

    1. Run rviz
    2. Select "map" as fixed frame and add required views [TF, LaserScan(/scan_filtered), Map (/map), PoseArray (/particlecloud)]
    3. Select "2D Pose Estimate" and click on the map where the robot is actually located
    4. Use joystick to rotate and translate such that the PoseArray arrows align

  .. note::
      Link to the ROS wiki for amcl: 
      http://wiki.ros.org/amcl