#include "unknown_map_path_finder.hpp"
#include "path_finder_utils.hpp"
#include "autonomy_simulator.hpp"

void updateGraphFromSensorData(Graph& graph, const std::vector<int>& updatedIndices,
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

void depthFirstPathFindingRec(Graph& graph, std::vector<bool>& visited, Vertex destination,
        Vertex current, uint8_t roverPoseR, ros::Publisher roverMovePublisher) {
}

inline void depthFirstPathFinding(Graph& graph, Vertex destination, Vertex source,
        uint8_t roverPoseR, ros::Publisher roverMovePublisher) {
    
    std::vector<bool> visited(num_vertices(graph), false);
    depthFirstPathFindingRec(graph, visited, destination, source, roverPoseR, roverMovePublisher);
}

/////////////////////////

UnknownMapPathFinder::UnknownMapPathFinder(uint8_t heightDeltaThreshold):
    PathFinderManager(heightDeltaThreshold),
    _graph(Graph(GRID_SIZE*GRID_SIZE)) {
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
        _roverPoseY, _roverPoseR, GRID_SIZE);

    updateGraphFromSensorData(_graph, updatedIndices, _map, GRID_SIZE, GRID_SIZE, _heightDeltaThreshold);
}
