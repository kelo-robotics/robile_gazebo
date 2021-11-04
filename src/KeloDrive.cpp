
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
    std::string leftHubWheelTopic = name + std::string("_drive_left_hub_wheel_controller/command");
    std::string rightHubWheelTopic = name + std::string("_drive_right_hub_wheel_controller/command");
    _leftHubWheelVelPub = nh.advertise<std_msgs::Float64>(leftHubWheelTopic, 1);
    _rightHubWheelVelPub = nh.advertise<std_msgs::Float64>(rightHubWheelTopic, 1);
}

void KeloDrive::getPos(double& xPos, double& yPos, double& zPos) const {
    xPos = _xPos;
    yPos = _yPos;
    zPos = _zPos;
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
