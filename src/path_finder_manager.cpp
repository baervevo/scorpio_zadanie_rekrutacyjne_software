#include "path_finder_manager.hpp"
#include "path_finder.hpp"
#include "autonomy_simulator.hpp"

PathFinderManager::PathFinderManager(uint8_t heightDeltaThreshold):
    _roverPoseX(0),
    _roverPoseY(0),
    _roverPoseR(autonomy_simulator::RoverPose::ORIENTATION_NORTH),
    _goalX(0),
    _goalY(0),
    _map(std::vector<int8_t>(GRID_SIZE*GRID_SIZE, -1)),
    _graph(Graph(GRID_SIZE*GRID_SIZE)),
    _heightDeltaThreshold(heightDeltaThreshold) {
}

void PathFinderManager::setActivePose(std::array<int8_t, 3>& poseData) {
    _roverPoseX = poseData[0];
    _roverPoseY = poseData[1];
    _roverPoseR = poseData[2];
}