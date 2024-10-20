#include "known_map_path_finder.hpp"
#include "path_finder_utils.hpp"
#include "autonomy_simulator.hpp"

KnownMapPathFinder::KnownMapPathFinder(uint8_t heightDeltaThreshold, std::vector<int8_t>& mapData):
    PathFinderManager(heightDeltaThreshold) {

    _map = mapData;
    populateGraphBasedOnMap(_graph, _map, _heightDeltaThreshold, GRID_SIZE, GRID_SIZE);
}

bool KnownMapPathFinder::setGoal(std::pair<uint8_t, uint8_t>& goalData) {
    PathFinderManager::setGoal(goalData);

    if(_goalX != _roverPoseX || _goalY != _roverPoseY) {
        _activeRoute = createVertexStackFromBFS(
            _graph,
            coordinatesToMapDataIndex(_roverPoseX, _roverPoseY, GRID_SIZE),
            coordinatesToMapDataIndex(_goalX, _goalY, GRID_SIZE)
        );

        if(!_activeRoute.empty()) {
            ROS_INFO("Path found.");
            return true;
        } else {
            ROS_ERROR("No valid path was found!");
            return false;
        }
    }

    return false;
}

int KnownMapPathFinder::getNextMove() {
    if(_roverPoseX == _goalX && _roverPoseY == _goalY) {
        ROS_INFO("Reached goal!");
        return -1;
    }

    if(!_activeRoute.empty()) {
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
            destX = destCoordinates.first;
            destY = destCoordinates.second;
        }

        return determineMoveInstruction(_roverPoseX, _roverPoseY, _roverPoseR, destX, destY);
    } else {
        ROS_ERROR("Route stack is empty!");
        return -1;
    }
}