
/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Sushant Chavan
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

#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "robile_gazebo/KeloDrive.h"

KeloDrive::KeloDrive(ros::NodeHandle& nh, const std::string& name, double xPos, double yPos, double zPos, double pivotOrientation) : 
_name(name), 
_xPos(xPos),
_yPos(yPos), 
_zPos(zPos), 
_pivotOrientation(pivotOrientation) {
    std::string leftHubWheelTopic = name + std::string("_left_hub_wheel_controller/command");
    std::string rightHubWheelTopic = name + std::string("_right_hub_wheel_controller/command");
    _leftHubWheelVelPub = nh.advertise<std_msgs::Float64>(leftHubWheelTopic, 1);
    _rightHubWheelVelPub = nh.advertise<std_msgs::Float64>(rightHubWheelTopic, 1);
}

void KeloDrive::getPos(double& xPos, double& yPos, double& zPos) const {
    xPos = _xPos;
    yPos = _yPos;
    zPos = _zPos;
}

void KeloDrive::setPivotOrientation(double orientation) {
    _pivotOrientation = orientation;
}

visualization_msgs::Marker KeloDrive::getPivotMarker() const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    getPos(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    tf2::Quaternion quat;
    quat.setRPY(0, 0, _pivotOrientation);
    quat.normalize();
    marker.pose.orientation = tf2::toMsg(quat);

    marker.scale.x = 0.25;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    return marker;
}

void KeloDrive::setHubWheelVelocities(double leftAngVel, double rightAngVel) {
    std_msgs::Float64 msg;

    msg.data = leftAngVel;
    _leftHubWheelVelPub.publish(msg);

    msg.data = rightAngVel;
    _rightHubWheelVelPub.publish(msg);
}
