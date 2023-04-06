Worksheet 01: ROS2 Nodes, Topics, Frames (TF) and URDF
====================================

System setup:
-------------
The instructions to set-up the system can be found [here](https://robile-amr.readthedocs.io/en/rolling/getting_started.html)
Please make sure the version of the documentation is "rolling". It can be found in the left bottom corner of the documentation page. If any issue is found while setting up the system, please contact the teaching assistant.

Recap of running the simulation:
-------------------------------
> Note:  
      - Make sure to source the `/opt/ros/rolling/setup.bash` in every new terminal.
      - Buid all the packages as mentioned in the `Getting Started` section of the documentation above.
      - Verify if the branch is set to `rolling` in all the repositories. If not, please switch to `rolling` branch. The branch can be switched by running `git checkout rolling` command in the repository directory.
      - Please deactivate `conda` environment if it is activated. The `conda` environment can be deactivated by running `conda deactivate` command in the terminal. To do this globally, set the conda configuration by running `conda config --set auto_activate_base false` command in the terminal. This will deactivate the `conda` environment by default.

### Running robile platform in simulation:

* The simulation can be launched by running the following command in the terminal:
```bash 
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```
* In the rviz, select the configuration from the File->Open Config menu. The configuration file can be found in the `robile_gazebo/config/rviz/robile_rolling.rviz` path.

* In the gazebo, if the platform is not spawned, please stop the simulation by `Ctrl+C` and launch the simulation again. (Please do not use `Ctrl+Z` to stop the simulation. It will not kill the script but will put it in the background.)

* If the launch file stops by itself or even after pressing `Ctrl+C`, please verify if any process named `gz_server` is running. If it exists, then kill the process by using the PID associated with the process. In the below command, replace the `<PID>` with the PID of the process. The PID can be found by running `ps` command in the terminal:
```bash
ps
kill -9 <PID>
```

* Great job! You have successfully launched the simulation. Now, let's move on to the exercises.
* If you are facing any issue, please post your queries in the lea forum of AMR.

Exercise-01: Move the Robile platform in simulation
----------------------------------------------------

> Objective: to understand the basic structure of ROS2 nodes, topics and messages.

Send velocity commands to the robile platform to move in a particular pattern. The velocity commands can be sent to the `\cmd_vel` topic. This topic accepts `geometry_msgs/msg/Twist` message. The `geometry_msgs/msg/Twist` message has two fields `linear` and `angular`. The `linear` field is of type `geometry_msgs/msg/Vector3` and `angular` field is of type `geometry_msgs/msg/Vector3`. The `geometry_msgs/msg/Vector3` message has three fields `x`, `y` and `z`. The `x` and `y` fields in `linear` field are used to specify the linear velocity in x and y directions respectively. The `z` field in `angular` field is used to specify the angular velocity about z-axis. The `geometry_msgs/msg/Twist` message definition can be found [here](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)

### Move in circle:

Move the robile platform in a circular pattern. Write your implementation in the `robile_control_simple` package placed under `Robile/robile_demos/` path to send velocity commands to the robile platform. Build the package from your workspace in a new terminal. Before building, please source the setup.bash file of ROS as described in the `Getting Started` section of the documentation. After building, source the local_setup.bash file (`source ~/WS_Name/install/local_setup.bash`). Run the `robile_control_simple` node to move the platform in a circular pattern. The node can be run by executing the following command in the terminal, once the simulation is launched and rviz is running:

```bash
ros2 run robile_control_simple circular_motion_publisher
```

### Move in square:

Once the platform is successfully moving in a circular pattern, create your own node by following the tutorial from [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). Setup the package.xml, setup.py and your node file similar to the `robile_control_simple` node. Make the robot move a square pattern instead of circular pattern. Please feel free to use additional python libraries if required.


Exercise-02: Move the Robile platform to a goal
------------------------------------------------

> Objective: to understand different available frames in the system and to transform the goal position from one frame to another.

The robot is associated with different frames which simplifies the task of specifying tasks with different scopes. In the simulation, different frames can be viewed under the `tf` section in rviz. The most frequently used frames are `odom`, `base_link` and `base_laser`. When the robot driver is started, the location of the robot will be fixed as `odom` frame. The `base_link` of the robot is assigned to the centroid of the robot measured in 2D plane. The `base_laser` frame is assigned to the laser sensor of the robot. When the robot driver is started, both `odom` and `base_link` will be overlapping. Generally the `odom` frame will be considered as a fixed frame and `base_link` will be used to provide velocity commands. The environment will be perceived with respect to the `base_laser` frame. This can be transformed to the `base_link` to decide the direction of motion and it can be transformed to the `odom` frame to get the global position of the goals or obstacles. This allows to continuously update velocity commands based on the the perception data at the first instance and allows to plan path without active perception.

### Move to a goal:

<img src="../bitmaps/robile_ws0_ex_02" width="350" height="250">

Consider a situation where the robot has to reach a particular location where an object is placed. The robot perceives it with respect to the `base_laser` frame, the robot has to be move towards the goal location. The goal location can be transfomed with respect to the `base_link` frame and also with respect to the `odom` frame (as they might not be overlapping if the robot has already moved after starting its drivers). The transformed goal with respect to the `base_link` can be used to provide the direction for `cmd_vel` and the transformed goal with respect to the `odom` frame can be used to monitor if the robot's `base_link` has reached the goal. In the above image, the dotted line arrow represents the perceived goal with respect to the `base_laser` frame. The solid line arrow represents the transformed goal with respect to the `base_link` frame. As `cmd_vel` is published with respect to the `base_link`, the robot has to be moved along the solid line arrow to reach the goal location. 


Write your implementation in the `robile_reach_goal` package placed under `Robile/robile_demos/` path to complete the following tasks:
* Transform the goal location from `base_laser` frame to `base_link` frame.
* Transform the goal location from `base_laser` frame to `odom` frame.
* Use the transformed goal location with respect to `base_link` frame to decide values for the `\cmd_vel` topic.
* Use the transformed goal location with respect to `odom` frame to monitor if the robot has reached the goal location.

Excercise-03: Add a new sensor to the Robile platform
------------------------------------------------------

> Objective: to understand the structure of a sensor plugin and to add a new sensor to the platform.