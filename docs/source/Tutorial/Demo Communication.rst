.. _architecture:

Tutorial for establishing communication 
========================================

This tutorial is a demonstration for establishing communication and controlling the robot. 

* Connecting to the wifi network **Robile5G** and in terminal connect to the robot (please add these as aliases in ~/.bashrc for convenience)

  .. code-block:: bash

    -  ssh -x studentkelo@192.168.0.101   [while connecting to Robile1]     
    -  ssh -x studentkelo@192.168.0.102   [while connecting to Robile2]     
    -  ssh -x studentkelo@192.168.0.103   [while connecting to Robile3]     
    -  ssh -x studentkelo@192.168.0.104   [while connecting to Robile4] 

  Password for all three platforms: "area5142"  

* For example, to add alias of the to connect to the terminal of Robile1, add the following line in your **.bashrc** file:
  
  .. code-block:: bash

      alias robile1='ssh -x studentkelo@192.168.0.101'
    
  .. note::
    In the bashrc of all robiles, the environmnet variable **ROS_DOMAIN_ID** is set to the respective robile number. For example, for Robile4, it is set to the number **4**.
    
* Launch the real-robot (from the terminal where you are accessing the terminal of robot)

  .. code-block:: bash

      ros2 launch robile_bringup robot.launch.py

* Launch the robot in simulation

  .. code-block:: bash

      ros2 launch robile_gazebo gazebo_4_wheel.launch.py

* To visualise the robot in rviz2 while using real robot, run the following command in a new terminal. Make sure that the **ROS_DOMAIN_ID** is set to the respective robile number. For example, for Robile4, it is set to the number **4**. Please don't leave space between the equal sign and the number.

  .. code-block:: bash

      export ROS_DOMAIN_ID=4
      rviz2    

  To load the rviz2 config file, click on the **Open Config** button in the top left corner of the rviz2 window and select the file **robile.rviz** from folowing path.

  .. code-block:: bash

      robile_gazebo/config/robile.rviz
       
