.. _getting_started:

Getting started
###############

.. _install_ubuntu:

Install Ubuntu
==============

The repository and its related components have been tested under the following Ubuntu distributions:

- Ubuntu 20.04 (ROS2 version: Rolling) 

If you do not have a Ubuntu distribution on your computer you can download it `here <https://releases.ubuntu.com/focal/>`_.

.. _git_version_control:

Git - Version Control
=====================

* Install Git Software

  Install the Git core components and some additional GUI's for the version control:

  .. code-block:: bash

    sudo apt-get install git-core gitg gitk

* Set Up Git

  Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

  .. code-block:: bash

     git config --global user.name "Your Name Here"

  Git also saves your email address into the commits you make.

  .. code-block:: bash

     git config --global user.email "your-email@youremail.com"


* GIT Tutorial
  If you have never worked with git before, we recommend to go through the
  `basic git tutorial <http://excess.org/article/2008/07/ogre-git-tutorial/>`_.

.. _robot_operating_system:

ROS - Robot Operating System
============================

* Install ROS2

  The repository has been tested successfully with the following ROS2 rolling distribution.
  Instructions to install ROS2 can be found `here <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html>`_.

  .. note::
    For convenience, adding the following line in the .bashrc file (located in home location) is recommended: <source /opt/ros/rolling/setup.bash> (without the < >).
    If multiple ROS distributions are being used, then aliases can be used source particular distribution in the .bashrc file.

* ROS2 Tutorials

  If you have never worked with ROS before, it is recommended to go through
  `the beginner tutorials provided by ROS2 <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_.

.. _setup_colcon_workspace:

Dependencies
==================

Install following packages on which are the dependencies of the repositories in the HBRS-AMR group:

  .. code-block:: bash

    pip install --upgrade sphinx docutils py_trees
    sudo apt-get install ros-rolling-gazebo-ros ros-rolling-turtlebot3-gazebo ros-rolling-xacro ros-rolling-tf2-geometry-msgs ros-rolling-turtle-tf2-py ros-rolling-tf2-tools ros-rolling-tf-transformations ros-rolling-joint-state-publisher-gui ros-rolling-joint-state-publisher ros-rolling-joy-linux ros-rolling-urg-node-msgs

Setup Colcon Workspace
=========================

*   Colcon is the build system used by ROS2. It is recommended to install it as well by following the instructions `here <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_. 

*   Create a new workspace (make sure to source the setup.bash file in every terminal if not already added to the .bashrc file). If you are using multiple ROS distributions, then it is recommended to create a separate workspace for each distribution.

    .. code-block:: bash

      mkdir -p ~/colcon_ws_AMR/src
      cd ~/colcon_ws_AMR
      colcon build

* Clone and build the repositories from the HBRS-AMR group

  First of all you have to clone the repository.

  .. code-block:: bash

    cd ~/colcon_ws_AMR/src 
    git clone -b rolling https://github.com/HBRS-AMR/Robile.git
    git clone -b rolling https://github.com/HBRS-AMR/robile_description.git

  Build the packages and source the workspace before continuing (as robile_gazebo depends on robile_description).

  .. code-block:: bash

    cd ~/colcon_ws_AMR
    colcon build
    source ~/colcon_ws_AMR/install/local_setup.bash

  Continue with cloning the other repositories:

  .. code-block:: bash
    
    cd ~/colcon_ws_AMR/src
    git clone -b rolling https://github.com/HBRS-AMR/robile_gazebo.git
    git clone -b rolling https://github.com/HBRS-AMR/kelo_tulip.git

  Then go on with installing further external dependencies:

  .. code-block:: bash

    cd ~/colcon_ws_AMR/src

    git clone -b devel https://github.com/splintered-reality/py_trees_ros.git  

  Build the packages and source the workspace after cloning required repositories.

  .. code-block:: bash

    cd ~/colcon_ws_AMR
    colcon build
    source ~/colcon_ws_AMR/install/local_setup.bash

  .. note::
    While kelo_tulip package is building and if it looks stuck, please enter the password of your system and press enter. This allows to build with sudo privileges.

  If no errors appear everything is ready to use. Great job!
