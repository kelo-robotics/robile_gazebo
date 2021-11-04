
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include "robile_gazebo/Robile.h"

Robile::Robile(ros::NodeHandle& nh):
_nh(nh),
_cmdVelX(0.0),
_cmdVelY(0.0),
_cmdVelA(0.0) {
    _jointStatesSubscriber = _nh.subscribe("/joint_states", 1, &Robile::jointStatesCallBack, this);
    _pivotMarkersPublisher = _nh.advertise<visualization_msgs::MarkerArray>("/pivot_markers", 1);
}

void Robile::setCmdVel(double vx, double vy, double va) {
    _cmdVelX = vx;
    _cmdVelY = vy;
    _cmdVelA = va;
}

void Robile::publishPivotMarkers() const {
    visualization_msgs::MarkerArray markerArray;
    int markerId = 0;
    for (const auto& drive: _drives) {
        visualization_msgs::Marker marker = drive.second.getPivotMarker();
        marker.id = markerId++;
        markerArray.markers.push_back(marker);
    }
    _pivotMarkersPublisher.publish(markerArray);
}

void Robile::step() {
    // Implement Controller here
}

void  Robile::initDrives(const std::map<std::string, double>& pivotJointData) {
    if (!_drives.empty())
        return;

    ros::Time now = ros::Time::now();
    for (auto& joint: pivotJointData) {
        std::string driveName = getKeloDriveName(joint.first);
        std::string pivotLink = std::string("/") + driveName + std::string("_drive_pivot_link");
        if (!_tfListener.waitForTransform("/base_link", pivotLink, now, ros::Duration(2.0))) {
            ROS_WARN("Did not receive transform from /base_link to %s", pivotLink.c_str());
            _drives.clear();
            return;
        }
        tf::StampedTransform transform;
        _tfListener.lookupTransform("/base_link", pivotLink, now, transform);

        _drives.insert(std::make_pair(driveName,
                                      KeloDrive(_nh,
                                                driveName, 
                                                transform.getOrigin().x(), 
                                                transform.getOrigin().y(), 
                                                transform.getOrigin().z(),
                                                joint.second)));
    }

    ROS_INFO("Initialized %d Kelo drives", _drives.size());
}

void Robile::jointStatesCallBack(const sensor_msgs::JointState& msg) {
    const std::vector<std::string>& jointNames = msg.name;
    const std::vector<double>& jointPositions = msg.position;

    if (jointNames.empty() || jointNames.size() != jointPositions.size()) {
        return;
    }

    std::map<std::string, double> pivotJointData;
    std::string pivotJointNameIdentifier = "pivot_joint";
    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::string jointName = jointNames[i];
        if (jointName.find(pivotJointNameIdentifier) != std::string::npos) {
            // Normalize the pivot angle in range [0, 2*PI]
            double pivotAngle = jointPositions[i] - (int(jointPositions[i] / (2*M_PI)) * 2 * M_PI);
            pivotJointData[jointName] = pivotAngle;
        }
    }

    if (_drives.empty()) {
        initDrives(pivotJointData);
    } else {
        setPivotOrientations(pivotJointData);
    }
}

void Robile::setPivotOrientations(const std::map<std::string, double>& pivotJointData) {
    for (const auto& joint: pivotJointData) {
        std::string driveName = getKeloDriveName(joint.first);
        if (_drives.find(driveName) == _drives.end()) {
            ROS_ERROR("Cannot set pivot orientation for drive %s", driveName.c_str());
            continue;
        }
        KeloDrive& drive = _drives.at(driveName);
        drive.setPivotOrientation(joint.second);
    }
}

std::string Robile::getKeloDriveName(const std::string& jointName) {
    return jointName.substr(0, jointName.find("_drive_"));
}
