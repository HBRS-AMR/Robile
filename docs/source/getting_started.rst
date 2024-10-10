.. _getting_started:

Getting Started
###############

.. _install_ubuntu:

Install Ubuntu
==============

  The repositories and their related components that we will be using are tested under the following Ubuntu distributions:

  **ROS2 Humble: Ubuntu 22.04**

  If you do not have a Ubuntu distribution on your computer you can download it `here <https://ubuntu.com/download/alternative-downloads>`_.

.. _git_version_control:

Git - Version Control
=====================

  **Installing Git**

  Install the Git core components and some additional GUI's for the version control.

  .. code-block:: bash

    sudo apt-get install git-core gitg gitk git

  **Setting up Git**

  To check if you have already configured user credentials, run the following commands. 
  
  .. code-block:: bash

    git config --global user.name
    git config --global user.email

  If it returns an empty string, then it's recommended that you tell git your name and email address,
  so that it can properly label the commits you make. If it returns your name and email address,
  then you can skip this step.

  .. code-block:: bash

     git config --global user.name "Your Name Here"

  Git also saves your email address into the commits you make.

  .. code-block:: bash

     git config --global user.email "your-email@youremail.com"


  **Git Tutorial**
  
  If you are new to git, or just want to brush up on the commands, we recommend that you go through 
  the `basic git tutorial <http://excess.org/article/2008/07/ogre-git-tutorial/>`_ and also follow 
  the `practice_git <https://github.com/kvnptl/practice_git>`_  repository, which you may have used 
  in your foundation course.

.. _robot_operating_system:

ROS - Robot Operating System
============================

  **Install ROS**

  Please follow `ROS2 Humble installation <http://docs.ros.org/en/humble/Installation.html>`_ instructions, if you have not already installed ROS2. 

  .. note::
    For convenience, adding the following line in the .bashrc file (located in home location) is recommended: 
    *source /opt/ros/humble/setup.bash*. If multiple ROS distributions are being used, 
    then aliases can be set for individual distributions in the .bashrc file.

  **ROS Tutorials**

  To refresh your ROS concepts, it is recommended to refer to
  `the beginner tutorials <https://docs.ros.org/en/humble/Tutorials.html>`_ provided by ROS2.

  It is recommended to revise the basic understanding of **nodes** and **topics**, and practicing **writing basic nodes and launch files**.

.. _setup_catkin_workspace:

Dependencies
============

  Install following packages:

  .. code-block:: bash

    pip install --upgrade sphinx docutils
    sudo apt-get install ros-humble-gazebo-ros ros-humble-turtlebot3-gazebo ros-humble-xacro ros-humble-tf2-geometry-msgs ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations ros-humble-joint-state-publisher-gui ros-humble-joint-state-publisher ros-humble-joy-linux ros-humble-urg-node ros-humble-urg-node-msgs ros-humble-rosbag2-* ros-humble-ros2bag


Setup Colcon Workspace
======================

  Colcon is the build system used by ROS2. The following instructions to install colcon and creating a workspace 
  is derived from `this <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_ link.

  **Installing Colcon**

  .. code-block:: bash

    sudo apt install python3-colcon-common-extensions

  **Creating a Workspace**

  .. code-block:: bash

    source /opt/ros/humble/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    
  .. note::
    The *~/workspace_name/* directory (in the above example, it is *~/ros2_ws/*) is termed as **root** of the workspace. 
    In ROS2, while building the packages in a workspace, *colcon build* should be run from the root of the workspace.

Cloning Repositories from HBRS-AMR group
========================================

  First, you have to clone and build the **Robile** and **robile_description** repositories. The `Robile <https://github.com/HBRS-AMR/Robile.git>`_ 
  repository contains the core packages to drive the robot and the `robile_description <https://github.com/HBRS-AMR/robile_description.git>`_ repository 
  contains the description of the robot.

  .. code-block:: bash

    cd ~/ros2_ws/src  
    git clone -b ros2 https://github.com/HBRS-AMR/Robile.git
    git clone -b ros2 https://github.com/HBRS-AMR/robile_description.git

  Build the packages and source the workspace before continuing (as robile_gazebo depends on robile_description).

  .. code-block:: bash

    cd ~/ros2_ws
    colcon build
    source ~/ros2_ws/install/setup.bash

  We will clone **robile_gazebo**, **robile_navigation**, and **robile_interfaces** repositories. The `robile_gazebo <https://github.com/HBRS-AMR/robile_gazebo.git>`_ 
  repository contains the simulation related components of the robot, the `robile_navigation <https://github.com/HBRS-AMR/robile_navigation.git>`_ repository 
  contains the navigation related software for the robot, and the `robile_interfaces <https://github.com/HBRS-AMR/robile_interfaces.git>`_ 
  repository is the repository used to store the custom messages used in other repositories.

  .. code-block:: bash

    cd ~/ros2_ws/src
    git clone -b ros2 https://github.com/HBRS-AMR/robile_gazebo.git
    git clone -b ros2 https://github.com/HBRS-AMR/robile_navigation.git
    git clone https://github.com/HBRS-AMR/robile_interfaces.git

  In the **robile_navigation/config/nav2_params.yaml**, update the complete path to `smacPlannerLattice` under `planner_server-> GridBased->lattice_filepath`.

  Once cloning above repositories and editing the `nav2_params.yaml`, we will build the packages and source the workspace after cloning the above repositories.
  
  .. code-block:: bash

    cd ~/ros2_ws
    colcon build
    source ~/ros2_ws/install/setup.bash

  If no errors appear everything is ready to use. Great job!
