# robile_behaviors

## Overview

Different behaviors of Robile using `py_trees` are implemented in this package.

## Structure

```
robile_behaviors
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____launch
     |    |_____wall_following.launch
     |    |
     |____scripts
     |    |_____behavior_tree
     |                      |     __init__.py
     |                      |     behaviors.py 
     |                      |_____utils.py 
     |____src
          |    wall_following
          |    autonomous_mapping
          |    battery_voltage_publisher
          |____marker_line
```

## Description

`marker_line` node helps to visualize the extracted lines in `rviz` and the `battery_voltage_publisher` node initialises the `mileage` topic over which the battery voltage can be published. As volatage of the battery is not directly accesible, the `mileage` topic is used to publish voltage commands to the behavior tree, which is used as a trigger for all behaviors.

### `wall_following` behavior

![wall_following BT](doc/figures/wall_following_BT.png)

`wall_following` node consists of structuring of behavior tree using the behaviors imported from `behaviors.py`. It will find the closest wall, moves towards it, aligns to it and keep following the wall.


### `autonomous_mapping` behavior

![autonomous_mapping BT](doc/figures/autonomous_mapping_BT.png)

1. `autonomous_mapping` node consists of structuring of behavior tree using the behaviors imported from `behaviors.py`. It is built as an extension of `wall_following` behavior to autonomously map an environment using wall following behavior.

2. Set the `fixed frame` to `map` frame in rviz (from the console on the left) after mapping an environment.


## Instructions to run scripts:

1. Refer to the `getting stared` section from the [documentation](https://robile-amr.readthedocs.io/en/latest/getting_started.html) and the instructions from the [assignmnet file](https://github.com/HBRS-AMR/WS22_Assignment_BehaviorTree) for the initial setup of the repository (if not completed already).

2. To install the navigation stack from ROS, run the following command.
    ```
    sudo apt-get install ros-noetic-navigation
    ```

3. If initial setup was already completed, pull from the `main` branch to get the `robile_behavior` package. Build the package and source the `setup.bash` from catkin workspace.

4. To run a behavior (eg: `wall_follwing`, `autonomous_mapping`) in simuation, run the following command, and publish the battery voltage to the `mileage` topic to trigger the behavior tree.
    ```
    roslaunch <name_of_behavior> wall_following.launch  
    rostopic pub /mileage std_msgs/Float32 "data: 50.0" -r10
    ```

5. `rviz` will open automatically, and once it is open select the dropdown `File`, select `Open Config`. Locate `robile_wall_following.rviz` from the `Robile/robile_gazebo/config/rviz/` path and click on the `close wihtout saving` button in the pop-up.

6. To visualise the behavior tree and the blackboard, please run the following command in new terminal:
    ```
    rqt_py_trees
    py-trees-tree-watcher  
    py-trees-blackboard-watcher
    ```