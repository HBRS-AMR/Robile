.. _architecture:

Tutorial for creating a map 
============================

This tutorial is a demonstration for creating a  map of an environment using "gmapping" algorithm of ROS navigation stack. 



* Launch the robot

  .. code-block:: bash

      roslaunch robile_bringup robot.launch

* Run 2D SLAM

  .. code-block:: bash

      roslaunch robile_navigation_demo gmapping.launch

  .. note::

      The map is built using the front laser's only

* Run the robot using joystick to map an environment
  
  .. note::

      Export ROS_MASTER_URI to wifi ip-address and export ROS_IP to your system ip-address.
      Now run "rviz" in the terminal and add 'map' and 'laser_scan' displays. Change the topics accordingly.

* Run the map saver

  After traversing the map, go to the map configuration directory

  .. code-block:: bash

      roscd robile_default_env_config

  By using `ls` you can see several folders corresponding to existing environments.
  You can either use an existing map or create a new one:

  .. code-block:: bash

      mkdir [map_name]
      cd [map_name]

  And then run:

  .. code-block:: bash

      rosrun map_server map_saver -f <map_name_local> --free <free_thresh> --occ <occupied_thresh>

  This will create two files: a `map_name_local.pgm` and `map_name_local.yml`. (considering map as [map_name_local])

  Finally, to use the map that you just created you need to check which map will be loaded by the navigation stack:

  .. code-block:: bash

      echo $ROBOT_ENV

  If you need to change it:

  .. code-block:: bash

      export ROBOT_ENV=[map_name_local]

  .. note::

      Usually the `.rosc` script is used to set the environment, among other variables

  .. note::
      Link to the ROS wiki for gmapping  : 
      http://wiki.ros.org/gmapping