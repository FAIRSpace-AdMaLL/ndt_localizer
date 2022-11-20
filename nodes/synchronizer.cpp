#include <iostream>
#include <iomanip> 
#include <algorithm>
#include "synchronizer.h"

Synchronizer::Synchronizer(unsigned int buffer_size) : queue_size_(buffer_size) {}

void Synchronizer::insertPose(const RobotPoseStampted& pose) {
    buffer_.emplace_back(pose);

    while(buffer_.size() > queue_size_) {
        buffer_.pop_front();
    }
    std::cout << "queue_size: " << buffer_.size() << std::endl;;
}

bool Synchronizer::synchronize(RobotPoseStampted& result_pose, double query_time) {
    std::cout << "queue size: " << buffer_.size();
    std::cout << std::setprecision(15) << "query_time: " << query_time << " buffer_.back().time_stamp: " << buffer_.back().time_stamp << std::endl;
    if(buffer_.size() >= 2 && query_time <= buffer_.back().time_stamp) {
        std::deque<RobotPoseStampted>::iterator iter_low = std::lower_bound(buffer_.begin(), buffer_.end(), query_time, [](const RobotPoseStampted& a, const double b) {return a.time_stamp < b;});
        std::deque<RobotPoseStampted>::iterator iter_prev = iter_low;
        std::deque<RobotPoseStampted>::iterator iter_post = iter_low+1;
        if(interpolatePose(*iter_prev, *iter_post, result_pose, query_time)) {
            std::cout << "synchronization succeed!";
            return true;
        }
    }
    return false;
}

bool Synchronizer::interpolatePose(const RobotPoseStampted& prev, const RobotPoseStampted& post, RobotPoseStampted& result, double query_time) {
    result = post;
    return true;
}