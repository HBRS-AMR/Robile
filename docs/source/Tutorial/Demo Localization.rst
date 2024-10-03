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

  If ROBOT_NAME environment variable is not set, set it to the robot name you want to use. For example,

    .. code-block:: bash
    
        export ROBOT_NAME=robile1               [when using Robile1 real-robot]
        export ROBOT_NAME=robile_gazebo         [when using Robile in simulation]

  Before running map_server node, build the robile_navigation package from the source directory of the workspace and source the setup.bash file. When built, the map will be stored in the share folder in the install directoy of the workspace. This makes the map to be accessible to the map_server node

  .. code-block:: bash

      cd ~/ros2_ws
      colcon build --packages-select robile_navigation
      source install/setup.bash


  Run map_server node in new terminal of your PC. If you are using real-robot, make sure to set the environment variable **ROS_DOMAIN_ID** to respective robot id while using **your** terminal. Eg: while using Robile1, `export ROS_DOMAIN_ID=1`

  .. code-block:: bash

      ros2 launch robile_navigation robile_nav2_bringup.launch.py


  If your application requires to subscribe to the map topic, then the qos profile has to be set to *qos_profile_sensor_data*. An example of creating a subscriber is as follows:

    .. code-block:: cpp
    
        self.map_subscriber = self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos_profile=rclpy.qos.qos_profile_sensor_data);

**Running AMCL**

  Launch AMCL node

  .. code-block:: bash

      ros2 launch robile_navigation localization.launch.py

  Localize the robot

  1. Run rviz2 in new terminal and select this config file: `robile_navigation/config/robile_ros2_nav.rviz`  
  2. Select "2D Pose Estimate" placed in the top bar of rviz visulizer and click drag the arrow on the map where the robot is actually located
  3. Use joystick o teleop_twist_keyboard to rotate and translate such that the PoseArray arrows align
  4. The particles will converge to the actual location of the robot and the robot will be localized
  5. Keeping the terminal where localization node is running open, use a new terminal to run the navigation node, which is discussed in the next section
