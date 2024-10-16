#include <path_finder.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "path_finder_utils.hpp"

void PathFinder::setGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& goal) {
    if(_illegalState) {
        return;
    }

    _goalX = goal->x;
    _goalY = goal->y;
    _active = true;
}

void PathFinder::roverPoseCallback(const autonomy_simulator::RoverPose::ConstPtr& pose) {
    if(_illegalState) {
        return;
    }

    _roverPoseX = pose->x;
    _roverPoseY = pose->y;
    _roverPoseR = pose->orientation;
}

void PathFinder::roverMoveCallback(const ros::TimerEvent&) {
    // The idea for pathfinding is a feedback loop between the goal and current position.
    // If we completely trust the /rover/pose topic to publish the true current position of our rover
    // it may be safer to continuously take into account the current position when planning the next move
    // rather than creating a queue of moves which will be "blindly" executed.
    // This could come in handy in real-world scenarios where an instruction doesn't necessarily yield the
    // expected result.

    if(_illegalState) {
        return;
    }

    if(_roverPoseX == _goalX && _roverPoseY == _goalY) {
        _active = false;
    }

    if(_active) {
        std_msgs::UInt8 msg;

        msg.data = determineMoveInstruction(_roverPoseX, _roverPoseY, _roverPoseR, _goalX, _goalY);

        _roverMovePublisher.publish(msg);
    }
}

PathFinder::PathFinder():
    _illegalState(false),
    _roverPoseX(0),
    _roverPoseY(0),
    _roverPoseR(autonomy_simulator::RoverPose::ORIENTATION_NORTH),
    _goalX(0),
    _goalY(0),
    _active(false),
    _nh(ros::NodeHandle("path_finder")),
    _roverMovePublisher(_nh.advertise<std_msgs::UInt8>("/rover/move", 1)),
    _roverMovePublisherTimer(_nh.createTimer(
        ros::Duration(0.1),
        std::bind(&PathFinder::roverMoveCallback, this, std::placeholders::_1),
        false,
        false)),
    _roverPoseSubscriber(_nh.subscribe("/rover/pose", 1, &PathFinder::roverPoseCallback, this)),
    _setGoalSubscriber(_nh.subscribe("/set_goal", 1, &PathFinder::setGoalCallback, this)) {
        
}

void PathFinder::start() {
    _roverMovePublisherTimer.start();
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_finder");

    //ROS_INFO("I'm alive!");

    PathFinder pathFinder;
    pathFinder.start();
    return 0;
}