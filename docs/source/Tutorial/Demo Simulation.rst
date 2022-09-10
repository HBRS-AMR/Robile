.. _architecture:

Tutorial for running the implementation of wall following behaviour in simulation 
================================

This tutorial guides to verify the implementation of wall following robot behaviour in simulation. 
The model of the corridor in front of the @home lab (C069) is considered for this simulation

* Launch the robot model (robile 3) in gazebo:

  .. code-block:: bash

      roslaunch robile_gazebo robile3_platform.launch

* Run wall_follower node

  .. code-block:: bash

      roslaunch robile_navigation_demo wall_follower.launch

  .. note::
      The launch file will run the wall follower node from github repository. To run the implementation
      from local system, the python file can be run from the terminal    


  .. note::    
    Topics:  
      - /cmd_vel: the topic to publish velocity commands       
      - /scan_filtered: the topic where the filtered laser scan data is being published    