#pragma once

#include <deque>
#include "common.h"

class Synchronizer {
public:
Synchronizer(unsigned int buffer_size);
~Synchronizer() = default; 

bool synchronize(RobotPoseStampted& pose, double query_time);
void insertPose(const RobotPoseStampted& pose);
bool interpolatePose(const RobotPoseStampted& prev, const RobotPoseStampted& post, RobotPoseStampted& result, double query_time);

private:
std::deque<RobotPoseStampted> buffer_;
unsigned int queue_size_ = 2;

};

