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

#ifndef KELO_DRIVE_H
#define KELO_DRIVE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/**
 * @brief Class to store information about and manage an individual Kelo drive
 * that is part of a ROBILE platform
 *
 */
class KeloDrive {
  public:
    /**
     * @brief Construct a new Kelo Drive object
     *
     * @param nh Node handle for the ros node managing the platform controller
     * @param name Name of the Kelo drive
     * @param xPos Position of the kelo drive along the x-axis of the base_link
     * @param yPos Position of the kelo drive along the y-axis of the base_link
     * @param zPos Position of the kelo drive along the z-axis of the base_link
     * @param pivotOrientation Initial orientation of the kelo drive pivot with
     * respect to the base_link
     */
    KeloDrive(ros::NodeHandle& nh, const std::string& name, double xPos,
              double yPos, double zPos, double pivotOrientation);

    /**
     * @brief Destroy the Kelo Drive object
     *
     */
    virtual ~KeloDrive() {}

    /**
     * @brief Get the position of the kelo drive w.r.t base_link
     *
     * @param xPos
     * @param yPos
     * @param zPos
     */
    void getPos(double& xPos, double& yPos, double& zPos) const;

    /**
     * @brief Set the latest pivot orientation of the kelo drive
     *
     * @param orientation Latest pivot orientation w.r.t base_link
     */
    void setPivotOrientation(double orientation);

    /**
     * @brief Get the latest pivot orientation of the kelo drive
     *
     * @return double Latest pivot orientation w.r.t base_link
     */
    double getPivotOrientation() const { return _pivotOrientation; }

    /**
     * @brief Get an RViz marker object representing the pose of the kelo drive
     * pivot for debugging purposes
     *
     * @return visualization_msgs::Marker marker representing current pose of
     * the kelo drive pivot
     */
    visualization_msgs::Marker getPivotMarker() const;

    /**
     * @brief Set the desired angular velocities for each of the two hub wheels
     * of the kelo drive
     *
     * @param leftAngVel Angular velocity for the left hub wheel in rad/s
     * @param rightAngVel Angular velocity for the right hub wheel in rad/s
     */
    void setHubWheelVelocities(double leftAngVel, double rightAngVel);

  protected:
    /**
     * @brief Name of the kelo drive
     * 
     */
    std::string _name;

    /**
     * @brief Position of the kelo drive along the x-axis of the base_link
     * 
     */
    double _xPos;

    /**
     * @brief Position of the kelo drive along the y-axis of the base_link
     * 
     */
    double _yPos;

    /**
     * @brief Position of the kelo drive along the z-axis of the base_link
     * 
     */
    double _zPos;

    /**
     * @brief Orientation of the kelo drive pivot with respect to the base_link
     * 
     */
    double _pivotOrientation;

    /**
     * @brief ROS publishers for setting the desired velocity for the left hub 
     * wheel
     * 
     */
    ros::Publisher _leftHubWheelVelPub;

    /**
     * @brief ROS publishers for setting the desired velocity for the right hub 
     * wheel
     * 
     */
    ros::Publisher _rightHubWheelVelPub;
};

#endif // KELO_DRIVE_H
