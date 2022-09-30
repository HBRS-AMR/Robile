/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Sushant Chavan
 * Walter Nowak
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "robile_gazebo/RobilePlatformController.h"

RobilePlatformController* platformController;

void cmdVelCallBack(const geometry_msgs::Twist& msg) {
    if (platformController) {
        platformController->setCmdVel(msg.linear.x, msg.linear.y, msg.angular.z);
    }
}

void joyCallback(const sensor_msgs::Joy& joy) {
    if (platformController) {
        platformController->setCmdVel(joy.axes[0], joy.axes[1], joy.axes[2]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robile_gazebo_controller");
    ros::NodeHandle nh;
    ros::Subscriber cmdVelSubscriber = nh.subscribe("/cmd_vel", 10, cmdVelCallBack);
    ros::Subscriber joySubscriber = nh.subscribe("/joy", 1000, joyCallback);

    // Load max linear velocity
    double platformMaxLinVel;
    if (!ros::param::get("/platform_max_lin_vel", platformMaxLinVel)) {
        platformMaxLinVel = 1.0;
    }

    // Load max angular velocity
    double platformMaxAngVel;
    if (!ros::param::get("/platform_max_ang_vel", platformMaxAngVel)) {
        platformMaxAngVel = 1.0;
    }

    platformController = new RobilePlatformController(nh);
    platformController->setMaxPlatformVelocity(platformMaxLinVel, platformMaxAngVel);

    ros::Rate loopRate(20);
    while (ros::ok()) {
        ros::spinOnce();
        platformController->step();
        platformController->publishOdomToBaseLinkTF();
        platformController->publishOdom();
        platformController->publishPivotMarkers();

        loopRate.sleep();
    }

    return 0;
}
