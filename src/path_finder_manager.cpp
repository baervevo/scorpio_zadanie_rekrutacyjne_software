#include "path_finder_manager.hpp"
#include "path_finder.hpp"
#include "autonomy_simulator.hpp"

PathFinderManager::PathFinderManager(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight):
    _roverPoseX(0),
    _roverPoseY(0),
    _roverPoseR(autonomy_simulator::RoverPose::ORIENTATION_NORTH),
    _goalX(0),
    _goalY(0),
    _map(std::vector<int8_t>(gridWidth*gridHeight, -1)),
    _heightDeltaThreshold(heightDeltaThreshold),
    _gridWidth(gridWidth),
    _gridHeight(gridHeight) {
}

void PathFinderManager::setActivePose(std::array<int8_t, 3>& poseData) {
    _roverPoseX = poseData[0];
    _roverPoseY = poseData[1];
    _roverPoseR = poseData[2];
}