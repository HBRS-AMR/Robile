.. _architecture:

Tutorial for creating a map 
============================

This tutorial is a demonstration for creating a  map of an environment using "gmapping" algorithm of ROS navigation stack. 

* Connecting to the robot
  .. code-block:: bash

    -  ssh -x studentkelo@192.168.0.101   [Robile1]     
    -  ssh -x studentkelo@192.168.0.102   [Robile2]     
    -  ssh -x studentkelo@192.168.0.103   [Robile3]     
    -  ssh -x studentkelo@192.168.0.104   [Robile4] 

      Password for all three platforms: "area5142"  

* In every terminal of the platform, assign 'ROS_IP' to ip address of the robot. Eg: on robile1,  `export ROS_IP=192.168.0.1`.[this is already included in robot's `.bashrc` file. It is not required to run this.]

  .. code-block:: bash

        export ROS_IP=<robile_ip_address>

* Launch the robot (on real robot)

  .. code-block:: bash

      roslaunch robile_bringup robot.launch

* Launch the robot (in simulation)

  .. code-block:: bash

      roslaunch robile_gazebo robile3_platform.launch

* Run 2D SLAM

  .. code-block:: bash

      roslaunch robile_navigation_demo gmapping.launch

  .. note::

      The map is built using the front laser's only

* In simulation control the robot using teleop keys,
  
    .. code-block:: bash

        rosrun teleop_twist_keyboard teleop_twist_keyboard.py

* If `teleop_twist_keyboard` is not installed, 
  
    .. code-block:: bash

        sudo apt-get install ros-noetic-teleop-twist-keyboard

* Run the robot using joystick to map an environment
  
  - Export ROS_MASTER_URI to wifi ip-address and export ROS_IP to your system ip-address.  

  .. code-block:: bash

      export ROS_MASTER_URI=http://<robile_ip_address>:11311 && export ROS_IP=<your_system_ip_address>  

  - your_system_ip_address can be found by running 'ifconfig'. Please make sure you are connected to the same network as the robot(ROBILE5G).
      
  - Now run "rviz" in the same terminal. Add 'map' and 'laser_scan' displays and change the topics accordingly.

* Run the map saver

  After traversing the map, go to the map configuration directory

  .. code-block:: bash

      roscd robile_default_env_config/ros/

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

      export ROBOT_ENV=[map_name/map_name_local]  
      Eg: export ROBOT_ENV=team_m/corridor_iter_n  

  .. note::

      Usually the `.rosc` script is used to set the environment variable, among other variables

  .. note::
      Link to the ROS wiki for gmapping: 
      http://wiki.ros.org/gmapping