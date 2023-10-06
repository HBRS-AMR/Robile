.. _getting_started:

Getting started
###############

.. _install_ubuntu:

Install Ubuntu
==============

  The repository and its related components have been tested under the following Ubuntu distributions:

  **ROS2 Humble: Ubuntu 22.04**

  If you do not have a Ubuntu distribution on your computer you can download it `here <https://ubuntu.com/download/alternative-downloads>`_.

.. _git_version_control:

Git - Version Control
=====================

  **Installing Git**

  Install the Git core components and some additional GUI's for the version control.

  .. code-block:: bash

    sudo apt-get install git-core gitg gitk git

  **Seting up Git**

  To check if you have already configured user credentials, run the following commands. 
  If it returns empty string, then proceed with the next step. If not, skip the next step.
  
  .. code-block:: bash

    git config --global user.name
    git config --global user.email

  If you have not configured Git on your system, it's time to configure your settings. 
  To do this you need to open a new Terminal. First you need to tell git your name, 
  so that it can properly label the commits you make:

  .. code-block:: bash

     git config --global user.name "Your Name Here"

  Git also saves your email address into the commits you make.

  .. code-block:: bash

     git config --global user.email "your-email@youremail.com"


  **Git Tutorial**
  
  If you have never worked with git before, we recommend to go through the
  `basic git tutorial <http://excess.org/article/2008/07/ogre-git-tutorial/>`_ and 
  also a good tutorial can be found in the repository `practice_git <https://github.com/kvnptl/practice_git>`_, 
  which you might have used in your foundation course.

.. _robot_operating_system:

ROS - Robot Operating System
============================

  **Install ROS**

  The repository has been tested successfully with the ROS2 humble version.
  Please follow `ROS2 Humble installation <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ instructions, if you have not installed already.

  .. note::
    For convenience, adding the following line in the .bashrc file (located in home location) is recommended: 
    <source /opt/ros/humble/setup.bash> (without the < >). If multiple ROS distributions are being used, 
    then aliases can be set for individual distributions in the .bashrc file.

  **ROS Tutorials**

  If you have limited experience with ROS before, we recommend to go through
  `the beginner tutorials <https://docs.ros.org/en/humble/Tutorials.html>`_ provided by ROS2.

  In order to understand at least the different core components of ROS, understanding concepts of **nodes**, **topics**, and getting hands on experience on **writing basic nodes and launch files** are recommended.

.. _setup_catkin_workspace:

Dependencies
======================

  Install following packages:

  .. code-block:: bash

    pip install --upgrade sphinx docutils py_trees
    sudo apt-get install ros-humble-gazebo-ros ros-humble-turtlebot3-gazebo ros-humble-xacro ros-humble-tf2-geometry-msgs ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations ros-humble-joint-state-publisher-gui ros-humble-joint-state-publisher ros-humble-joy-linux ros-humble-urg-node ros-humble-urg-node-msgs


Setup Colcon Workspace
=========================

  Colcon is the build system used by ROS2. The following instructions to install colcon and creating a workspace 
  is derived from `this <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>_`` link.

  **Installing Colcon**

  .. code-block:: bash

    sudo apt install python3-colcon-common-extensions

  **Creating a Workspace**

  .. code-block:: bash

    source /opt/ros/humble/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/colcon_ws_AMR
    colcon build
    
  .. note::
    The *~/workspace_name/* directory (in the above example, it is *~/ros2_ws/*) is termed as **root** of the workspace. 
    In ROS2, while building the packages in a workspace, *colcon build* should be run from the root of the workspace.

Cloning Repositories from HBRS-AMR group
============================================

  First of all you have to clone and build the **Robile** and **robile_description** repositories. The [Robile](https://github.com/HBRS-AMR/Robile.git) 
  repository contains the core packages to drive the robot and the [robile_description](https://github.com/HBRS-AMR/robile_description.git) repository 
  contains the description of the robot.

  .. code-block:: bash

    cd ~/ros2_ws/src  
    git clone -b rolling https://github.com/HBRS-AMR/Robile.git
    git clone -b rolling https://github.com/HBRS-AMR/robile_description.git

  Build the packages and source the workspace before continuing (as robile_gazebo depends on robile_description).

  .. code-block:: bash

    cd ~/ros2_ws
    colcon build
    source ~/ros2_ws/install/setup.bash

  Cloning **robile_gazebo** and **kelo_tulip** repositories. The [robile_gazebo](https://github.com/HBRS-AMR/robile_gazebo.git) 
  repository contains the gazebo simulation of the robot and the [kelo_tulip](https://github.com/HBRS-AMR/kelo_tulip.git) repository 
  contains the platform controller for the robot.

  .. code-block:: bash

    cd ~/ros2_ws/src
    git clone -b rolling https://github.com/HBRS-AMR/robile_gazebo.git
    git clone -b rolling https://github.com/HBRS-AMR/kelo_tulip.git

  Cloning **py_trees_ros** repository. The [py_trees_ros](https://github.com/splintered-reality/py_trees_ros.git) 
  repository is a tool we use to structure and configure the behavior tree implementation for the robot.

  .. code-block:: bash

    cd ~/ros2_ws/src
    git clone -b devel https://github.com/splintered-reality/py_trees_ros.git

  Build the packages and source the workspace after cloning the above repositories.

  .. code-block:: bash

    cd ~/ros2_ws
    colcon build
    source ~/ros2_ws/install/setup.bash

  .. note::
    While kelo_tulip package is building (can be seen in terminal when *colcon build* is run) and if it looks stuck at around 80%, please enter the password of your system and press enter. Don't worry if you cannot see your system password being typed in thee terminal. After pressing *Enter*, the package will be built successfully. 

  If no errors appear everything is ready to use. Great job!