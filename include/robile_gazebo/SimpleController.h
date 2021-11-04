#include "robile_gazebo/KeloDrive.h"
#include <map>

std::map< std::string, std::pair< double, double > > getWheelVelocities(const std::map< std::string, KeloDrive > drives, double vx, double vy) {
    std::map< std::string, std::pair< double, double > > controlCommands;
    for (const auto &drive : drives) {
        std::string driveName = drive.first;

        double targetPivot = atan2(vx, vy);
        double pivotDelta = targetPivot - drive.second.getPivotOrientation();
        double pivotPgain = 5.0;

        double speed = sqrt(vx * vx + vy * vy);
        double speedPgain = 10.0;

        std::pair<double, double> wheelVelocities = std::make_pair(-pivotDelta * pivotPgain + speed * speedPgain,
                                                                    pivotDelta * pivotPgain + speed * speedPgain);

        controlCommands.insert(std::make_pair(driveName, wheelVelocities));
    }

    return controlCommands;
}
