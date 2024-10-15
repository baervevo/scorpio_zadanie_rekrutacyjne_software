#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"

class PathFinder {
    bool _illegalState;
    ros::NodeHandle _nh;

    // PathFinder information

    uint8_t _roverPoseX;
    uint8_t _roverPoseY;
    autonomy_simulator::RoverPose::_orientation_type _roverPoseR;

    uint8_t _goalX;
    uint8_t _goalY;
    bool _active;

    // SetGoal Subscriber
    ros::Subscriber _setGoalSubscriber;
    void setGoalCallback(const autonomy_simulator::SetGoal::ConstPtr&);

    // RoverMove Publisher
    ros::Publisher _roverMovePublisher;
    ros::Timer _roverMovePublisherTimer;
    void roverMoveCallback(const ros::TimerEvent&);

    // RoverPose Subscriber
    ros::Subscriber _roverPoseSubscriber;
    void roverPoseCallback(const autonomy_simulator::RoverPose::ConstPtr&);

 public:
    PathFinder();
    void start();
};