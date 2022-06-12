Worksheet 02: Map creation - Localization - Navigation
==============

Objectives
---------
* To creating and saving a map of an environment using laser scan data (Reference: http://wiki.ros.org/gmapping)
* To investigate limitations of mapping an environment  

* To localize in the map using amcl (Reference: http://wiki.ros.org/amcl)  
* To investigate limitations of localizing in an environment  

* To navigate to a specified goal after localization (Referene: http://wiki.ros.org/move_base)  
* To report the effects of important navigation parameters  

Tasks
----------

* Please setup your machine with Ubuntu 20.04 (else use Docker), ROS Noetic and setup the ROS environment 
    > Instructions to setup your machine can be followed from this tutorial   
https://robile-amr.readthedocs.io/en/main/getting_started.html

### Task 01:
* Creating and saving a map of an environment using `gmapping` from ROS navigation stack   
    > Instructions for creating a map can be followed from this tutorial  
https://robile-amr.readthedocs.io/en/main/Tutorial/Demo%20Mapping.html

* Create a documentation on problems faced and the respective solutions you followed during entire mapping procedure

### Task 02:  
* Localizing in the map saved in previous task using `amcl` from ROS navigation stack  
    > Instructions for localizing in a map can be followed from this tutorial  
https://robile-amr.readthedocs.io/en/main/Tutorial/index.html#localization   

* Create a documentation on problems faced and the respective solutions you followed during the process of localization  

### Task 03:  
* Navigating the map using `move_base` from ROS navigation stack  
    > Instructions for navigating a map can be followed from this tutorial  
https://robile-amr.readthedocs.io/en/main/Tutorial/Demo%20Navigation.html   

* Create a documentation on effects of varying different navigation parameters on the cost map and navigation plans  

* This [documentation](https://arxiv.org/pdf/1706.09068.pdf) of parameter tuning for navigation would be helpful in understanding the effects of parameters on navigation