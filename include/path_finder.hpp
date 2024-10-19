#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <stack>
#include <boost/graph/adjacency_list.hpp>

#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/GetMap.h"
#include "autonomy_simulator/RoverMap.h"

using namespace boost;

typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;

class PathFinder {
    bool _illegalState;
    ros::NodeHandle _nh;

    // PathFinder information

    uint8_t _roverPoseX;
    uint8_t _roverPoseY;
    autonomy_simulator::RoverPose::_orientation_type _roverPoseR;

    uint8_t _goalX;
    uint8_t _goalY;

    std::vector<int8_t> _map;
    Graph _graph;
    std::stack<int> _activeRoute;
    int _heightDeltaThreshold;

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
};