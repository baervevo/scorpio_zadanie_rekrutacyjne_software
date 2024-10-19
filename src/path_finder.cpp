#include <path_finder.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <string>

#include "autonomy_simulator.hpp"
#include "path_finder_utils.hpp"

#include "autonomy_simulator/RoverMap.h"
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/GetMap.h"

using namespace boost;

void PathFinder::setGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& goal) {
    if(_illegalState) {
        ROS_ERROR("Illegal state in setGoalCallback");
        return;
    }

    // Resolve path based on goal coordinates and stored graph, ROS_ERROR if no valid path is found.

    _goalX = goal->x;
    _goalY = goal->y;

    ROS_INFO("New goal set: (%d,%d)", _goalX, _goalY);

    if(_goalX != _roverPoseX || _goalY != _roverPoseY) {
        _activeRoute = createVertexStackFromBFS(
            _graph,
            coordinatesToMapDataIndex(_roverPoseX, _roverPoseY, GRID_SIZE),
            coordinatesToMapDataIndex(_goalX, _goalY, GRID_SIZE)
        );

        if(!_activeRoute.empty()) {
            ROS_INFO("Path found.");
            _roverMovePublisherTimer.start();
        } else {
            ROS_ERROR("No valid path was found!");
        }
    }
}

void PathFinder::roverPoseCallback(const autonomy_simulator::RoverPose::ConstPtr& pose) {
    if(_illegalState) {
        ROS_ERROR("Illegal state in roverPoseCallback");
        return;
    }

    _roverPoseX = pose->x;
    _roverPoseY = pose->y;
    _roverPoseR = pose->orientation;
}

void PathFinder::roverMoveCallback(const ros::TimerEvent&) {
    // For now used only for known map traversal.

    if(_illegalState) {
        ROS_ERROR("Illegal state in roverMoveCallback");
        return;
    }

    if(_roverPoseX == _goalX && _roverPoseY == _goalY) {
        _roverMovePublisherTimer.stop();
        ROS_INFO("Goal reached!");
    } else if(!_activeRoute.empty()) {
        // Not entirely elegant but stack top access is O(1) and I can't think of an implementation
        // that forgoes the if statement without implementing a different way of dealing with rotation
        // within a single square in order to proceed. Having immediate access to roverPose within this scope
        // would help a bunch.
        std::pair<int, int> destCoordinates = mapDataIndexToCoordinates(_activeRoute.top(), GRID_SIZE);
        int destX = destCoordinates.first;
        int destY = destCoordinates.second;

        if(_roverPoseX == destX && _roverPoseY == destY) {
            _activeRoute.pop();

            destCoordinates = mapDataIndexToCoordinates(_activeRoute.top(), GRID_SIZE);
            int destX = destCoordinates.first;
            int destY = destCoordinates.second;
        }

        std_msgs::UInt8 msg;
        msg.data = determineMoveInstruction(_roverPoseX, _roverPoseY, _roverPoseR, destX, destY);

        _roverMovePublisher.publish(msg);
    } else {
        // Shouldn't ever enter here.

        ROS_ERROR("Route stack is empty!");
        _roverMovePublisherTimer.stop();
    }
}

void PathFinder::roverSensorCallback(const autonomy_simulator::RoverMap::ConstPtr& sensorData) {
    std::vector<int8_t> sensorDataTable(sensorData->data);
    std::vector<int> updatedIndices = updateMapBasedOnSensorData(_map, sensorDataTable, _roverPoseX,
        _roverPoseY, _roverPoseR, GRID_SIZE);

    updateGraphFromSensorData(_graph, updatedIndices, _map, GRID_SIZE, GRID_SIZE, _heightDeltaThreshold);
}

PathFinder::PathFinder():
    _illegalState(false),
    _roverPoseX(0),
    _roverPoseY(0),
    _roverPoseR(autonomy_simulator::RoverPose::ORIENTATION_NORTH),
    _goalX(0),
    _goalY(0),
    _map(std::vector<int8_t>(GRID_SIZE*GRID_SIZE, -1)),
    _graph(Graph(GRID_SIZE*GRID_SIZE)),
    _heightDeltaThreshold(0),
    _retrieveMapData(_nh.param(std::string("retrieve_map_data"), false)),
    _nh(ros::NodeHandle("path_finder")),
    _roverMovePublisher(_nh.advertise<std_msgs::UInt8>("/rover/move", 1)),
    _roverMovePublisherTimer(_nh.createTimer(
        ros::Duration(0.1),
        std::bind(&PathFinder::roverMoveCallback, this, std::placeholders::_1),
        false,
        false)),
    _roverPoseSubscriber(_nh.subscribe("/rover/pose", 1, &PathFinder::roverPoseCallback, this)),
    _setGoalSubscriber(_nh.subscribe("/set_goal", 1, &PathFinder::setGoalCallback, this)),
    _getMapServiceClient(_nh.serviceClient<autonomy_simulator::GetMap>("/get_map")),
    _roverSensorSubscriber(_nh.subscribe("/rover/sensor", 1, &PathFinder::roverSensorCallback, this)),
    _roverMapPublisher(_nh.advertise<std_msgs::UInt8>("/rover/map", 1)) {
        
}

void PathFinder::start() {
    int temp;
    _nh.param(std::string("height_delta_threshold"), temp, 0);
    _heightDeltaThreshold = temp;

    if(_retrieveMapData) {
        if(!_getMapServiceClient.waitForExistence(ros::Duration(5.0))) {
            ROS_ERROR("/get_map service unavailable!");
            _illegalState = true;
        }

        autonomy_simulator::GetMap getMapService;

        if(!_illegalState && _getMapServiceClient.call(getMapService)) {
            ROS_INFO("Map data retrieved.");

            _map = getMapService.response.data;
            
            populateGraphBasedOnMap(_graph, _map, _heightDeltaThreshold, GRID_SIZE, GRID_SIZE);
        } else {
            ROS_ERROR("Unable to retrieve map data.");
            _illegalState = true;
        }   
    }

    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_finder");

    ROS_INFO("Path finder starting");    

    PathFinder pathFinder;
    pathFinder.start();

    ROS_INFO("Path finder started.");

    return 0;
}