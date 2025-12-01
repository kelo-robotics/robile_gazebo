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

#include "robile_gazebo/RobilePlatformController.hpp"

RobilePlatformController::RobilePlatformController(rclcpp::Node::SharedPtr nh) : 
_nh(nh), 
_drives(), 
_initialized(false),
_tfBuffer(std::make_shared<tf2_ros::Buffer>(nh->get_clock())),
_tfListener(std::make_shared<tf2_ros::TransformListener>(*_tfBuffer)),
_cmdVelX(0.0), 
_cmdVelY(0.0), 
_cmdVelA(0.0),
_odomDuration(rclcpp::Duration::from_seconds(1.0)) 
{
    _jointStatesSub = nh->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&RobilePlatformController::jointStatesCallBack, this, std::placeholders::_1));
    _odomSub = nh->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&RobilePlatformController::odomCallBack, this, std::placeholders::_1));
    _tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
    _pivotMarkersPublisher = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/pivot_markers", 1);
    _hubWheelsPublisher = nh->create_publisher<std_msgs::msg::Float64MultiArray>("robile_hub_wheels_controller/commands", 1);
    setOdomFrequency(10.0);
}

void RobilePlatformController::setCmdVel(double vx, double vy, double va) {
    _cmdVelX = vx;
    _cmdVelY = vy;
    _cmdVelA = va;
}

void RobilePlatformController::setMaxPlatformVelocity(double linearVel, double angularVel) {
    _controller.setPlatformMaxLinVelocity(linearVel);
    _controller.setPlatformMaxAngVelocity(angularVel);
}

void RobilePlatformController::publishPivotMarkers() const {
    if (_pivotMarkersPublisher->get_subscription_count() > 0) {
        visualization_msgs::msg::MarkerArray markerArray;
        int markerId = 0;
        for (const auto &drive : _drives) {
            visualization_msgs::msg::Marker marker = drive.second.getPivotMarker();
            marker.header.frame_id = "base_link";
            marker.header.stamp = _nh->now();
            marker.id = markerId++;
            markerArray.markers.push_back(marker);
        }
        _pivotMarkersPublisher->publish(markerArray);
    }
}

void RobilePlatformController::setOdomFrequency(double frequency) {
    _odomDuration = rclcpp::Duration::from_seconds(1.0 / frequency);
}

void RobilePlatformController::step() {
    if (_initialized) {
        _controller.setPlatformTargetVelocity(_cmdVelX, _cmdVelY, _cmdVelA);
        _controller.calculatePlatformRampedVelocities();
        std::vector<double> allWheelVelocities;

        for (const auto &drive : _drives) {
            const std::string &driveName = drive.first;
            int wheelNumber = _wheelConfigs[driveName].ethercatNumber;
            float setpoint1, setpoint2;
            _controller.calculateWheelTargetVelocity(
            wheelNumber, drive.second.getPivotOrientation(), setpoint1, setpoint2);
            allWheelVelocities.push_back(-setpoint1);
            allWheelVelocities.push_back(-setpoint2);
        }
        std_msgs::msg::Float64MultiArray msg;
        msg.data = allWheelVelocities;
        _hubWheelsPublisher->publish(msg);
  }
}

void RobilePlatformController::initDrives(const std::map<std::string, double> &pivotJointData) {
    if (!_drives.empty())
    return;

    rclcpp::Time now = rclcpp::Clock().now();
    for (auto &joint : pivotJointData) {
        std::string driveName = getRobileBrickName(joint.first);
        std::string pivotLink = driveName + std::string("_drive_pivot_link");
        RCLCPP_INFO(_nh->get_logger(), "Checking transform for: %s", pivotLink.c_str());
        RCLCPP_INFO(_nh->get_logger(), "Transform OK for drive: %s", driveName.c_str());
        if (!_tfBuffer->canTransform("base_link", pivotLink, rclcpp::Time(0), rclcpp::Duration(std::chrono::seconds(2)))) {
            RCLCPP_WARN(_nh->get_logger(), "Did not receive transform from base_link to %s", pivotLink.c_str());
            _drives.clear();
            return;
        }
        geometry_msgs::msg::TransformStamped transform;
        transform = _tfBuffer->lookupTransform("base_link", pivotLink, rclcpp::Time(0));
        _drives.insert(std::make_pair(driveName, KeloDrive(driveName, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, joint.second)));
    }
    std::vector<kelo::WheelConfig> wheelConfigsVector;
    int wheelNumber = 0;
    double zDummy = 0;
    for (const auto &drive : _drives) {
        kelo::WheelConfig wc;
        wc.ethercatNumber = wheelNumber;
        drive.second.getPos(wc.x, wc.y, zDummy);
        wc.a = 0;
        _wheelConfigs[drive.first] = wc;
        wheelConfigsVector.push_back(wc);
        wheelNumber++;
    }

    _controller.initialise(wheelConfigsVector);
    _initialized = true;
    RCLCPP_INFO(_nh->get_logger(), "Initialized %lu Kelo drives", _drives.size());
}

void RobilePlatformController::jointStatesCallBack(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const std::vector<std::string> &jointNames = msg->name;
    const std::vector<double> &jointPositions = msg->position;

    if (jointNames.empty() || jointNames.size() != jointPositions.size()) {
        return;
    }

    std::map<std::string, double> pivotJointData;
    std::string pivotJointNameIdentifier = "drive_pivot_joint";

    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::string jointName = jointNames[i];
        if (jointName.find(pivotJointNameIdentifier) != std::string::npos) {
            double pivotAngle = jointPositions[i] - (int(jointPositions[i] / (2 * M_PI)) * 2 * M_PI);
            pivotJointData[jointName] = pivotAngle;
        }
    }
    if (_drives.empty()) {
        initDrives(pivotJointData);
    } else {
        setPivotOrientations(pivotJointData);
    }
}

void RobilePlatformController::setPivotOrientations(const std::map<std::string, double> &pivotJointData) {
    for (const auto &joint : pivotJointData) {
        std::string driveName = getRobileBrickName(joint.first);
        if (_drives.find(driveName) == _drives.end()) {
            RCLCPP_ERROR(_nh->get_logger(), "Cannot set pivot orientation for drive %s", driveName.c_str());
            continue;
        }
        KeloDrive &drive = _drives.at(driveName);
        drive.setPivotOrientation(joint.second);
    }
}

std::string RobilePlatformController::getRobileBrickName(const std::string &jointName) {
    return jointName.substr(0, jointName.find("_drive_"));
}

void RobilePlatformController::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    _odomMsg = *msg;
}

void RobilePlatformController::publishOdomToBaseLinkTF() {
    if (_initialized) {
        tf2_msgs::msg::TFMessage tfMsg;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "odom";
        transform.header.stamp = _nh->now();
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = _odomMsg.pose.pose.position.x;
        transform.transform.translation.y = _odomMsg.pose.pose.position.y;
        transform.transform.translation.z = _odomMsg.pose.pose.position.z;
        transform.transform.rotation = _odomMsg.pose.pose.orientation;
        tfMsg.transforms.push_back(transform);
        _tfBroadcaster->sendTransform(transform);
    }
}