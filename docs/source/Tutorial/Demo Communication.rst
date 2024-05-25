.. _architecture:

Tutorial for Establishing Communication 
========================================

We will look into establishing communication between the robot and the computer. In ROS2, we need to configure the network settings for reliable communication. 
This allows the robot and the computer to communicate with each other. We will then proceed to ssh into the robot and launch the drivers on the robot

**Network Configuration**

  Communication concepts of ROS to users by using existing middleware DDS. 
  This DDS middleware has different implementations. 
  To know more about this please refer to `information on DDS <https://design.ros2.org/articles/ros_on_dds.html>`_ 
  and `information on different implementations <https://design.ros2.org/articles/ros_middleware_interface.html>`_.
  Some of the common DDS implementations are **Fast DDS**, **Connext DDS**, **RTI DDS** etc.

  Each of these implementations have different different logics to establish communication between the robot and the computer.
  Sometimes the network interface of the computer is not able to communicate with the network interface of the robot. So it 
  is a good practice to configure the network settings to establish communication between the robot and the computer.
  Here we specify on which network interface the communication should happen. For example, in the below configuration, we
  specify `wlp3s0` as the network interface name which is connected to the same network as the robot


  Copy the content below and save it in a file. For example, the file can be named as **ros2_network_config.xml** and can be saved in the home directory or the ros2 workspace
  
  .. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>CustomTcpTransport</transport_id>
                <type>TCPv4</type>
                <interfaceWhiteList>
                    <address>wlp3s0</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="CustomTcpTransportParticipant">
            <rtps>
                <useBuiltinTransports>false</useBuiltinTransports>
                <userTransports>
                    <transport_id>CustomTcpTransport</transport_id>
                </userTransports>
            </rtps>
        </participant>
    </profiles>

  Once connecting to **Robile5G** network, run `ip a` command from your terminal. Replace the network 
  interface name **wlp3s0** in the above file with the network interface name from which your system is 
  connected to **Robile5G** network. In your **.bashrc** file, add the following lines at the end of the 
  file. Please make sure that the path to the `ros2_network_config`` file is correct. By setting `RMW_IMPLEMENTATION` as `rmw_fastrtps_cpp`, 
  we are setting the DDS implementation to use as the **Fast DDS**

  .. code-block:: bash

      export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_network_config.xml
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

**SSH-ing into Robile (while using real-robot)**
  
  Make sure you have connected to the wifi network **Robile5G**. In one of terminals, ssh to the robot (please add these as aliases in ~/.bashrc for convenience)

  .. code-block:: bash

      ssh -x studentkelo@192.168.0.101   [**while connecting to Robile1**]     
      ssh -x studentkelo@192.168.0.102   [**while connecting to Robile2**]     
      ssh -x studentkelo@192.168.0.103   [**while connecting to Robile3**]     
      ssh -x studentkelo@192.168.0.104   [**while connecting to Robile4**] 

  Password while connecting to Robiles: **area5142**

  For example, to add alias of the to connect to the terminal of Robile1, add the following line in your **.bashrc** file
  
  .. code-block:: bash

      alias robile1='ssh -x studentkelo@192.168.0.101'

  Now, you can connect to the terminal of Robile1 by typing **robile1** in the terminal, instead of typing the whole command
    
  .. note::

    In the bashrc of all robiles, the environmnet variable **ROS_DOMAIN_ID** is set to the respective robile number. For example, for Robile4, it is set to the number **4**
    
  To launch the real-robot, run following command from the terminal where you are accessing the terminal of robot

  .. code-block:: bash

      ros2 launch robile_bringup robot.launch.py

  To launch the robot in simulation, run the following command in a new terminal

  .. code-block:: bash

      ros2 launch robile_gazebo gazebo_4_wheel.launch.py

  .. note:: 

    To verify the communication between the robot and the computer, run the following command in a new terminal. If you are using real-robot, make sure that the **ROS_DOMAIN_ID** is set to the respective robile number

    .. code-block:: bash

        ros2 topic list

    If you get any error or if the entire list of topics is not printed, then run the following commands and try again to get the list of topics

    .. code-block:: bash

        ros2 daemon stop
        ros2 daemon start

**Visualizing Robile in Rviz2**

  To visualize the robot in Rviz2 while using real robot, run the following command in a new terminal. 
  Make sure that the **ROS_DOMAIN_ID** is set to the respective robile number. For example, for Robile4, 
  it is set to the number **4**. Please don't leave space between the equal sign and the number

  .. code-block:: bash

      export ROS_DOMAIN_ID=4
      rviz2    

  To load the rviz2 config file, click on the **Open Config** button in the top left corner of the rviz2 
  window and select the file **robile.rviz** from folowing path

  .. code-block:: bash

      robile_gazebo/config/robile.rviz