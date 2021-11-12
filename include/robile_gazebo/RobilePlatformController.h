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

#ifndef ROBILE_PLATFORM_CONTROLLER_H
#define ROBILE_PLATFORM_CONTROLLER_H

#include "kelo_tulip/VelocityPlatformController.h"
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include "KeloDrive.h"

/**
 * @brief Velocity controller for gazebo simulations of arbitrary
 * robile platform configurations
 *
 */
class RobilePlatformController {
  public:
    /**
     * @brief Construct a new Robile Platform Controller object
     *
     * @param nh Node handle for the ros node managing the platform controller
     */
    RobilePlatformController(ros::NodeHandle& nh);

    /**
     * @brief Set the linear and angular velocities for the robile platform
     *
     * @param vx Linear velocity (in m/s) along the positive x axis in robot
     * frame
     * @param vy Linear velocity (in m/s) along the positive y axis in robot
     * frame
     * @param va Angular velocity in (rad/s) around the positive z axis in robot
     * frame
     */
    void setCmdVel(double vx, double vy, double va);

    /**
     * @brief Set the maximum linear and angular velocities that the robile
     * platform could achieve. Any commanded velocities above these values will
     * be clipped.
     *
     * @param linearVel Max linear velocity the robile platform could achieve
     * @param angularVel Max angular velocity the robile platform could achieve
     */
    void setMaxPlatformVelocity(double linearVel, double angularVel);

    /**
     * @brief
     *
     */
    void step();

    /**
     * @brief Publish RViz markers for each active wheel's pivot pose
     *
     */
    void publishPivotMarkers() const;

    /**
     * @brief Set the frequency at which odometry messages are published. The
     * frequency must be less that or equal to the controller's frequency.
     *
     * @param frequency Frequency (in Hz) at which odometry messages should be
     * published
     */
    void setOdomFrequency(double frequency);

    /**
     * @brief Publish the latest odometry received from Gazebo on '/odom' topic
     *
     */
    void publishOdom();

    /**
     * @brief Publish the latest transfrom from odom to base_link on the '/tf'
     * topic
     *
     */
    void publishOdomToBaseLinkTF();

  protected:
    /**
     * @brief Callback function to receive and process the robot joint states
     * from Gazebo.
     *
     * The first message is used to get the positions of all Kelo drives
     * attached to the platform and initialize the Velocity controller from
     * kelo_tulip. The next messages are used to store the latest pivot
     * orientation of each Kelo drive.
     *
     * @param msg Ros message from Gazebo with the Joint state information
     */
    void jointStatesCallBack(const sensor_msgs::JointState& msg);

    /**
     * @brief Callback function to receive and process the robot link states
     * from Gazebo
     *
     * The base_link pose and twist published by Gazebo is stored and used to
     * construct the odometry message.
     *
     * @param msg Ros message from Gazebo with the Link state information
     */
    void gazeboLinkStatesCallBack(const gazebo_msgs::LinkStates& msg);

    /**
     * @brief Intialize the Kelo drive data structures and the Velocity
     * controller from kelo_tulip
     *
     * @param pivotJointData Map of Kelo drive name vs its latest pivot
     * orientation
     */
    void initDrives(const std::map< std::string, double >& pivotJointData);

    /**
     * @brief Set the Pivot Orientations for every Kelo drive in the robile
     * platform
     *
     * @param pivotJointData Map of Kelo drive name vs the latest pivot
     * orientation
     */
    void
    setPivotOrientations(const std::map< std::string, double >& pivotJointData);

    /**
     * @brief Extract the robile brick name from the pivot joint name
     *
     * @param jointName Joint name of the Kelo drive pivot
     * @return std::string Name of the robile brick
     */
    std::string getRobileBrickName(const std::string& jointName);

    ros::NodeHandle& _nh;
    std::map< std::string, KeloDrive > _drives;
    bool _initialized;
    tf::TransformListener _tfListener;
    ros::Subscriber _jointStatesSubscriber;
    ros::Subscriber _gazeboLinkStatesSubscriber;
    ros::Publisher _odomPublisher;
    ros::Publisher _odomTFPublisher;
    ros::Publisher _pivotMarkersPublisher;

    double _cmdVelX, _cmdVelY, _cmdVelA;

    nav_msgs::Odometry _odomMsg;
    ros::Duration _odomDuration;
    ros::Time _lastOdomPubTime;

    kelo::VelocityPlatformController _controller;
    std::map< std::string, kelo::WheelConfig > _wheelConfigs;
};

#endif // ROBILE_PLATFORM_CONTROLLER_H
