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

    platformController = new RobilePlatformController(nh);

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
