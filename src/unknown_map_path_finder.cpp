#include "unknown_map_path_finder.hpp"
#include "path_finder_utils.hpp"
#include "autonomy_simulator.hpp"

UnknownMapPathFinder::UnknownMapPathFinder(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight):
    PathFinderManager(heightDeltaThreshold, gridWidth, gridHeight),
    _graph(Graph(_gridWidth*_gridHeight)) {
}

int UnknownMapPathFinder::getNextMove() {
    return -1;
}

bool UnknownMapPathFinder::setGoal(std::pair<uint8_t, uint8_t>&) {
    return false;
}

void UnknownMapPathFinder::updateMapData(const autonomy_simulator::RoverMap::ConstPtr& sensorData) {
    std::vector<int8_t> sensorDataTable(sensorData->data);
    std::vector<int> updatedIndices = updateMapBasedOnSensorData(_map, sensorDataTable, _roverPoseX,
        _roverPoseY, _roverPoseR, _gridWidth);

    updateGraphFromSensorData(_graph, updatedIndices, _map, _gridWidth, _gridHeight, _heightDeltaThreshold);
}

void UnknownMapPathFinder::updateGraphFromSensorData(Graph& graph, const std::vector<int>& updatedIndices,
        const std::vector<int8_t>& mapData, const int mapWidth, const int mapHeight,
        const int heightDeltaThreshold) {

    for(int currentIndex : updatedIndices) {
        const auto& [x, y] = mapDataIndexToCoordinates(currentIndex, mapWidth);
        for(const auto& [nx, ny] : manhattanNeighbours(x, y)) {
            if(nx >= 0 && nx < mapWidth && ny >= 0 && ny < mapHeight) {
                int neighbourIndex = coordinatesToMapDataIndex(nx, ny, mapWidth);

                int heightDelta = std::abs(mapData[currentIndex] - mapData[neighbourIndex]);

                if(heightDelta <= heightDeltaThreshold) {
                    add_edge(currentIndex, neighbourIndex, graph);
                }
            }
        }
    }
}

void UnknownMapPathFinder::depthFirstPathFindingRec(Graph& graph, std::vector<bool>& visited,
        Vertex destination, Vertex current) {


}

void UnknownMapPathFinder::depthFirstPathFinding(Graph& graph, Vertex destination, Vertex source,
        std::function<void(int)> collector) {

    std::vector<bool> visited(num_vertices(graph), false);
    depthFirstPathFindingRec(graph, visited, destination, source);
}
