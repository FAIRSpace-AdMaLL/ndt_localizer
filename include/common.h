#pragma once

struct PoseDebug {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct RobotPose
{
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};


struct RobotPoseStampted {
    RobotPose pose;
    double time_stamp;
};