Worksheet03: Developing wall following robot application

Objectives
---------
* To develop a reactive wall following robot application using laser scan data 

Tasks
----------

### Task 01:
* Develop a reactive wall following robot. A sample structure of the implementation can be found [here](https://github.com/HBRS-AMR/Robile/blob/main/robile_navigation/robile_navigation_demo/ros/scripts/wall_follower.py). Please feel free to change the code structure to fit your approach   
* Command to run this node:  `roslaunch robile_navigation_demo wall_follower.launch`

Instructions
----------
* Visualisation of the baseline implementation of the wall following application can be found in [this](https://www.youtube.com/watch?v=Qx__mHaQDic) video. Corresponding [bag file](https://drive.google.com/file/d/1J1HzGtJwKeJXjJJ1eiDNKBoABTvVoexD/view?usp=sharing) can be used as reference to verify your implementation
    > Note:  
    
  .. code-block:: bash

      rosbag play <name_of_bag_file>
* Include the problems faced and the limitations observed and the corresponding solutions that you found while implementing the wall following application in the documentation