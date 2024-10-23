#include <path_finder.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <array>

#include "autonomy_simulator.hpp"
#include "path_finder_utils.hpp"

#include "autonomy_simulator/RoverMap.h"
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/GetMap.h"
#include "known_map_path_finder.hpp"

using namespace boost;

void PathFinder::setGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& goal) {
    if(_illegalState) {
        ROS_ERROR("Illegal state in setGoalCallback");
        return;
    }

    uint8_t goalX = goal->x;
    uint8_t goalY = goal->y;

    ROS_INFO("Setting new goal: (%d,%d)", goalX, goalY);

    std::pair<uint8_t, uint8_t> goalData = {
        goalX, goalY
    };

    if(_pathFinderManager->setGoal(goalData)) {
        _roverMovePublisherTimer.start();
    }
}

void PathFinder::roverPoseCallback(const autonomy_simulator::RoverPose::ConstPtr& pose) {
    if(_illegalState) {
        ROS_ERROR("Illegal state in roverPoseCallback");
        return;
    }

    std::array<int8_t, 3> poseData = {
        pose->x,
        pose->y,
        pose->orientation
    };

    _pathFinderManager->setActivePose(poseData);
}

void PathFinder::roverMoveCallback(const ros::TimerEvent&) {
    if(_illegalState) {
        ROS_ERROR("Illegal state in roverMoveCallback");
        return;
    }

    std_msgs::UInt8 msg;
    int instruction = _pathFinderManager->getNextMove();
    msg.data = instruction;

    if(instruction != -1) {
        _roverMovePublisher.publish(msg);
    } else {
        _roverMovePublisherTimer.stop();
    }
}

void PathFinder::roverSensorCallback(const autonomy_simulator::RoverMap::ConstPtr& sensorData) {
    if(_illegalState) {
        ROS_ERROR("Illegal state in roverSensorCallback");
        return;
    }

    _pathFinderManager->updateMapData(sensorData);
}

PathFinder::PathFinder():
    _illegalState(false),
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
    int heightDeltaThreshold;
    _nh.param(std::string("height_delta_threshold"), heightDeltaThreshold, 0);

    if(_retrieveMapData) {
        if(!_getMapServiceClient.waitForExistence(ros::Duration(5.0))) {
            ROS_ERROR("/get_map service unavailable!");
            _illegalState = true;
        }

        autonomy_simulator::GetMap getMapService;

        if(!_illegalState && _getMapServiceClient.call(getMapService)) {
            ROS_INFO("Map data retrieved.");

            std::vector<int8_t> mapData = getMapService.response.data;
            _pathFinderManager = new KnownMapPathFinder(heightDeltaThreshold, GRID_SIZE, GRID_SIZE, mapData);

            //ROS_INFO("Neighbouring square heights: %d, %d", mapData[1], mapData[50]);
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

    return 0;
}