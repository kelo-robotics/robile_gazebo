#ifndef ROBILE_H
#define ROBILE_H

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_listener.h>
#include "kelo_tulip/VelocityPlatformController.h"

#include "KeloDrive.h"

class Robile {
public:
    Robile(ros::NodeHandle& nh);

    void setCmdVel(double vx, double vy, double va);
    void step();
    void publishPivotMarkers() const;

    void setOdomFrequency(double frequency); // in Hz
    void publishOdom();
    void publishOdomToBaseLinkTF();

protected:
    void jointStatesCallBack(const sensor_msgs::JointState& msg);
    void gazeboLinkStatesCallBack(const gazebo_msgs::LinkStates& msg);
    void initDrives(const std::map<std::string, double>& pivotJointData);
    void setPivotOrientations(const std::map<std::string, double>& pivotJointData);
    std::string getKeloDriveName(const std::string& jointName);

    ros::NodeHandle& _nh;
    std::map<std::string, KeloDrive> _drives;
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
    std::map<std::string, kelo::WheelConfig> _wheelConfigs;
};

#endif //ROBILE_H
