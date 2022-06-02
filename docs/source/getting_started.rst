.. _getting_started:

Getting started
###############

.. _install_ubuntu:

Install Ubuntu
==============

The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Noetic: Ubuntu 20.04

If you do not have a Ubuntu distribution on your computer you can download it `here <https://ubuntu.com/download/alternative-downloads>`_.

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

.. .. _getting_started_docker:

.. Docker (Recommended)
.. ====================

.. To be able to use our software independent of the operating system and/or ROS 
.. distribution, it is recommended to use docker. You can follow 
.. :ref:`this tutorial <docker>` to use docker.

.. _robot_operating_system:

ROS - Robot Operating System
============================

* Install ROS

  The repository has been tested successfully with the following ROS distributions.
  Use the link behind a ROS distribution to get to the particular
  `ROS Noetic installation instructions <http://wiki.ros.org/noetic/Installation/Ubuntu>`_.

  .. note::
    Do not forget to update your .bashrc!

* ROS Tutorials

  If you have never worked with ROS before, we recommend to go through
  `the beginner tutorials provided by ROS <http://wiki.ros.org/ROS/Tutorials>`_.

  In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters").

.. _setup_catkin_workspace:

Setup catkin workspace
=========================

* Create a catkin workspace

  .. code-block:: bash

    source /opt/ros/noetic/setup.bash
    mkdir -p ~/catkin_ws_AMR/src  
    cd ~/catkin_ws_AMR
    catkin build

* Clone and compile the repository on ROBILE

  First of all you have to clone the repository.

  .. code-block:: bash

    cd ~/catkin_ws_AMR/src  
    git clone https://github.com/HBRS-AMR/Robile.git

  Then go on with installing further external dependencies:

  .. code-block:: bash

    cd ~/catkin_ws_AMR/src/Robile
    source ~/catkin_ws_AMR/devel/setup.bash

  The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.

  And finally compile the repository:

  .. code-block:: bash

    cd ~/catkin_ws_AMR
    catkin build

  If no errors appear everything is ready to use. Great job!