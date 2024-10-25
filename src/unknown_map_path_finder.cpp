#include "unknown_map_path_finder.hpp"
#include "path_finder_utils.hpp"
#include "autonomy_simulator.hpp"
#include "known_map_path_finder.hpp"
#include <limits>

UnknownMapPathFinder::UnknownMapPathFinder(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight):
    PathFinderManager(heightDeltaThreshold, gridWidth, gridHeight),
    _graph(Graph(_gridWidth*_gridHeight)),
    _visited(nullptr),
    _dfsStack(nullptr),
    _activeRoute(nullptr) {

    _map[0] = 0;
}

int UnknownMapPathFinder::getNextMove() {
    if(_goalX == _roverPoseX && _goalY == _roverPoseY) {
        ROS_INFO("Goal reached!");
        return -1;
    }

    // First we see if we haven't already discovered a path to the goal.

    int goalIndex = coordinatesToMapDataIndex(_goalX, _goalY, _gridWidth);
    int roverIndex = coordinatesToMapDataIndex(_roverPoseX, _roverPoseY, _gridWidth);

    
    if(_map[goalIndex] != -1 && _activeRoute->empty()) {
        if(_map[goalIndex] == std::int8_t(100)) {
            ROS_ERROR("Goal unreachable!");
            return -1;
        }

        *_activeRoute = KnownMapPathFinder::createVertexStackFromBFS(
            _graph,
            roverIndex,
            goalIndex
        );

        _nextVertex = -1;

        if(!_activeRoute->empty()) {
            ROS_INFO("Found direct path!");
        }
    }

    if(!_activeRoute->empty()) {
        // We reuse the stack top as the destination until we reach it, only then do we pop it.

        _nextVertex = _activeRoute->top();

        if(_nextVertex == roverIndex) {
            _activeRoute->pop();
            _nextVertex = _activeRoute->top();
        }
    } else {
        // If there was no direct path found then we DFS.

        if(_nextVertex == roverIndex || _nextVertex == -1) {
            _nextVertex = DFS();
        }
    }

    if(_nextVertex == -1) {
        ROS_ERROR("Goal unreachable!");
        return -1;
    }

    const auto& [nextX, nextY] = mapDataIndexToCoordinates(_nextVertex, _gridWidth);
    return determineMoveInstruction(_roverPoseX, _roverPoseY, _roverPoseR, nextX, nextY);
}

bool UnknownMapPathFinder::setGoal(std::pair<uint8_t, uint8_t>& goalData) {
    PathFinderManager::setGoal(goalData);       
    
    _nextVertex = -1;
    _visited = new std::vector<bool>(num_vertices(_graph), false);
    _dfsStack = new std::stack<Vertex>();
    _activeRoute = new std::stack<int>();

    _dfsStack->push(coordinatesToMapDataIndex(_roverPoseX, _roverPoseY, _gridWidth));

    return true;
}

Vertex UnknownMapPathFinder::DFS() {
    int currentIndex = coordinatesToMapDataIndex(_roverPoseX, _roverPoseY, _gridWidth);

    _visited->at(currentIndex) = true;
    std::pair<AdjacencyIterator, AdjacencyIterator> neighbours = adjacent_vertices(currentIndex, _graph);

    float minDist = std::numeric_limits<float>::max();
    int goalIndex = coordinatesToMapDataIndex(_goalX, _goalY, _gridWidth);
    Vertex nextVertex = -1;

    for(AdjacencyIterator adIt = neighbours.first; adIt != neighbours.second; ++adIt) {
        Vertex neighbour = *adIt;

        if(!_visited->at(neighbour)) { // Greedy approach but is effective because graph is dense. DFS guarantees we explore every possible path.
            float dist = euclideanDistance(neighbour, goalIndex, _gridWidth);

            if(dist <= minDist) {
                minDist = dist;
                nextVertex = neighbour;
            }
        }
    }

    if(nextVertex != -1) {
        _dfsStack->push(nextVertex);
        return nextVertex;
    } else if(_dfsStack->empty()) {
        return -1;
    } else {
        _dfsStack->pop();
        Vertex ret = _dfsStack->top();

        return ret;
    }
}

void UnknownMapPathFinder::updateMapData(const autonomy_simulator::RoverMap::ConstPtr& sensorData) {
    std::vector<int8_t> sensorDataTable(sensorData->data);

    std::vector<int> updatedIndices = updateMapBasedOnSensorData(_map, sensorDataTable, _roverPoseX,
        _roverPoseY, _roverPoseR, _gridWidth, _gridHeight);

    updateGraphFromSensorData(_graph, updatedIndices, _map, _gridWidth, _gridHeight, _heightDeltaThreshold);

    printStringToFile("/home/mariuszr/Desktop/map.txt", printMap(_map, _gridWidth));
}

void UnknownMapPathFinder::updateGraphFromSensorData(Graph& graph, const std::vector<int>& updatedIndices,
        const std::vector<int8_t>& mapData, const int mapWidth, const int mapHeight,
        const int heightDeltaThreshold) {

    for(int currentIndex : updatedIndices) {
        int8_t currentHeight = mapData[currentIndex];

        if(currentHeight != -1) {
            const auto& [x, y] = mapDataIndexToCoordinates(currentIndex, mapWidth);
            for(const auto& [nx, ny] : manhattanNeighbours(x, y)) {
                if(nx >= 0 && nx < mapWidth && ny >= 0 && ny < mapHeight) {
                    int neighbourIndex = coordinatesToMapDataIndex(nx, ny, mapWidth);
                    int8_t neighbourHeight = mapData[neighbourIndex];

                    if(neighbourHeight != -1) {
                        int heightDelta = std::abs(mapData[currentIndex] - mapData[neighbourIndex]);

                        if(heightDelta <= heightDeltaThreshold) {
                            add_edge(currentIndex, neighbourIndex, graph);
                        }
                    }
                }
            }
        }
    }
}