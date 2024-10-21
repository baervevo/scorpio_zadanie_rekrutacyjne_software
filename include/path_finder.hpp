#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <stack>

#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/GetMap.h"
#include "autonomy_simulator/RoverMap.h"
#include "path_finder_manager.hpp"

class PathFinder {
    bool _illegalState;
    ros::NodeHandle _nh;

    // PathFinder information

    PathFinderManager* _pathFinderManager;
    bool _retrieveMapData;

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

    // GetMap Service Client
    ros::ServiceClient _getMapServiceClient;

    // RoverSensor subscriber
    ros::Subscriber _roverSensorSubscriber;
    void roverSensorCallback(const autonomy_simulator::RoverMap::ConstPtr&);

    // RoverMap publisher
    ros::Publisher _roverMapPublisher;

 public:
    PathFinder();
    void start();

    ~PathFinder() {
        delete _pathFinderManager;
    }
};