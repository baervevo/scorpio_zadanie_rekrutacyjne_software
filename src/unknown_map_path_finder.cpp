#include "unknown_map_path_finder.hpp"
#include "path_finder_utils.hpp"
#include "autonomy_simulator.hpp"

UnknownMapPathFinder::UnknownMapPathFinder(uint8_t heightDeltaThreshold):
    PathFinderManager(heightDeltaThreshold) {
}

int UnknownMapPathFinder::getNextMove() {

}

void UnknownMapPathFinder::updateMapData(const autonomy_simulator::RoverMap::ConstPtr& sensorData) {
    std::vector<int8_t> sensorDataTable(sensorData->data);
    std::vector<int> updatedIndices = updateMapBasedOnSensorData(_map, sensorDataTable, _roverPoseX,
        _roverPoseY, _roverPoseR, GRID_SIZE);

    updateGraphFromSensorData(_graph, updatedIndices, _map, GRID_SIZE, GRID_SIZE, _heightDeltaThreshold);
}