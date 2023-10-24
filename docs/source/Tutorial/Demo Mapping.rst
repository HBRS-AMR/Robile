.. _architecture:

Tutorial for Mapping
====================

This tutorial is a demonstration for creating a  map of an environment using grid mapping from slam_toolbox. 

* Make sure you have connected to the wifi network **Robile5G** and in terminal ssh to the robot (please add these as aliases in ~/.bashrc for convenience)

  .. code-block:: bash

    -  ssh -x studentkelo@192.168.0.101   [while connecting to Robile1]     
    -  ssh -x studentkelo@192.168.0.102   [while connecting to Robile2]     
    -  ssh -x studentkelo@192.168.0.103   [while connecting to Robile3]     
    -  ssh -x studentkelo@192.168.0.104   [while connecting to Robile4] 

  - Password(for all robiles): **area5142**
  - It is recommended to ssh to robot only to launch the robot and to kill the robot. For all other purposes, it is recommended to use the terminal of **your** system.
  - Make sure to set the environment variable **ROS_DOMAIN_ID** to respective robot id while publishing/subscribing to any ros topics from **your** terminal. Eg: while using Robile1,  `export ROS_DOMAIN_ID=1`.

* To launch the real-robot, run following command from the terminal where you are accessing the terminal of robot.

  .. code-block:: bash

      ros2 launch robile_bringup robot.launch.py

* To launch the robot in simulation, run the following command in a new terminal.

  .. code-block:: bash

      ros2 launch robile_gazebo gazebo_4_wheel.launch.py

* Run mapping node, and move the robot around the environment

  .. code-block:: bash

      ros2 launch robile_navigation online_async.launch.py


* If using the robot in simulation, control the robot using teleop keys
  
    .. code-block:: bash

        ros2 run teleop_twist_keyboard teleop_twist_keyboard

* To visualize the robot in rviz2 while using real robot, run the following command in a new terminal. Make sure that the **ROS_DOMAIN_ID** is set to the respective robile number. For example, for Robile4, it is set to the number **4**. Please don't leave space between the equal sign and the number.

  .. code-block:: bash

      export ROS_DOMAIN_ID=4
      rviz2    

  To load the rviz2 config file, click on the **Open Config** button in the top left corner of the rviz2 window and select the file **robile.rviz** from folowing path.

  .. code-block:: bash

      robile_navigation/config/robile_ros2_nav.rviz

  - Now you should see the map being built in rviz2 while moving the robot around the environment.

* Run the map saver: After traversing the map, go to the **maps** directory under **robile_navigation** package and run the following command to save the map.

  .. code-block:: bash

      ros2 run nav2_map_server map_saver_cli -f map_name --ros-args -p save_map_timeout:=20.0

  This will create two files: a `map_name.pgm` and `map_name.yml`. Please use desired name for the map inplace of `map_name`. Now you can kill the mapping node.

  Finally, to use the map that you just created you need to check which map will be loaded by the navigation stack:

  .. code-block:: bash

      echo $ROBOT_ENV

  If it is not same as *map_name* that you have set, then you need to set the environment variable *ROBOT_ENV* to the map name that you want to use:

  .. code-block:: bash

      export ROBOT_ENV=map_name  
      Eg: export ROBOT_ENV=map_c069 

  .. note:: 
    Environment variables are only set for the current terminal session. ROBOT_ENV needs to be set in the terminal where you are launching the map_server node, which is explained under localization.

