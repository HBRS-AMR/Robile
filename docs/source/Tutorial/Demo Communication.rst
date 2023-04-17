.. _architecture:

Tutorial for creating communication 
====================================

This tutorial is a demonstration for establishing communication and controlling the robot. 

* Connecting to the robot (please add these as aliases in ~/.bashrc for convenience)
  .. code-block:: bash

    -  ssh -x studentkelo@192.168.0.101   [Robile1]     
    -  ssh -x studentkelo@192.168.0.102   [Robile2]     
    -  ssh -x studentkelo@192.168.0.103   [Robile3]     
    -  ssh -x studentkelo@192.168.0.104   [Robile4] 

      Password for all three platforms: "area5142"  

* In every terminal of the platform, assign 'ROS_IP' to ip address of the robot. Eg: on robile1,  `export ROS_IP=192.168.0.1`.[this is already included in robot's `.bashrc` file. It is not required to run this.]

  .. code-block:: bash

      export ROS_IP=<robile_ip_address>

* Launch the robot (on real robot, in the terminal where you are accessing the terminal of robot)

  .. code-block:: bash

      roslaunch kelo_tulip example_joypad.launch.py

* Launch the robot (in simulation)

  .. code-block:: bash

      roslaunch robile_gazebo gazebo_4_wheel.launch.py
