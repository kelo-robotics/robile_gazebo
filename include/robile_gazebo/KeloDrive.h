#ifndef KELO_DRIVE_H
#define KELO_DRIVE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class KeloDrive {
public:
    KeloDrive(ros::NodeHandle& nh, const std::string& name, double xPos, double yPos, double zPos, double pivotOrientation);
    virtual ~KeloDrive() {}

    bool init();

    void getPos(double& xPos, double& yPos, double& zPos) const;

    void setPivotOrientation(double orientation) { _pivotOrientation = orientation; }
    double getPivotOrientation() const { return _pivotOrientation; }
    visualization_msgs::Marker getPivotMarker() const;

    void setHubWheelVelocities(double leftAngVel, double rightAngVel);

protected:
    std::string _name;
 
    double _xPos, _yPos, _zPos;
    double _pivotOrientation;

    ros::Publisher _leftHubWheelVelPub, _rightHubWheelVelPub;
};

#endif //KELO_DRIVE_H
