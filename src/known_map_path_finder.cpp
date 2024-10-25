#include "known_map_path_finder.hpp"
#include "path_finder_utils.hpp"
#include "autonomy_simulator.hpp"

KnownMapPathFinder::KnownMapPathFinder(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight, std::vector<int8_t>& mapData):
    PathFinderManager(heightDeltaThreshold, gridWidth, gridHeight),
    _graph(Graph(_gridWidth*_gridHeight)) {

    _map = mapData;
    populateGraphBasedOnMap(_graph, _map, _heightDeltaThreshold, _gridWidth, _gridHeight);
}

bool KnownMapPathFinder::setGoal(std::pair<uint8_t, uint8_t>& goalData) {
    PathFinderManager::setGoal(goalData);

    if(_goalX != _roverPoseX || _goalY != _roverPoseY) {
        _activeRoute = createVertexStackFromBFS(
            _graph,
            coordinatesToMapDataIndex(_roverPoseX, _roverPoseY, _gridWidth),
            coordinatesToMapDataIndex(_goalX, _goalY, _gridWidth)
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
        std::pair<int, int> destCoordinates = mapDataIndexToCoordinates(_activeRoute.top(), _gridWidth);
        int destX = destCoordinates.first;
        int destY = destCoordinates.second;

        if(_roverPoseX == destX && _roverPoseY == destY) {
            _activeRoute.pop();

            destCoordinates = mapDataIndexToCoordinates(_activeRoute.top(), _gridWidth);
            destX = destCoordinates.first;
            destY = destCoordinates.second;
        }

        return determineMoveInstruction(_roverPoseX, _roverPoseY, _roverPoseR, destX, destY);
    } else {
        ROS_ERROR("Route stack is empty!");
        return -1;
    }
}

void KnownMapPathFinder::populateGraphBasedOnMap(Graph& graph, const std::vector<int8_t>& mapData,
        const int heightDeltaThreshold, const int8_t mapWidth, const int8_t mapHeight) {

    for(int y = 0; y < mapHeight; y++) {
        for(int x = 0; x < mapWidth; x++) {
            int currentIndex = coordinatesToMapDataIndex(x, y, mapWidth);
            for(const auto& [nx, ny] : manhattanNeighbours(x, y)) {
                if(nx >= 0 && nx < mapWidth && ny >= 0 && ny < mapHeight) {
                    int neighbourIndex = coordinatesToMapDataIndex(nx, ny, mapWidth);
                    int heightDelta = std::abs(mapData[currentIndex] - mapData[neighbourIndex]);

                    if(heightDelta <= heightDeltaThreshold) {
                        boost::add_edge(currentIndex, neighbourIndex, graph);
                    }
                }
            }
        }
    }
}

template <typename Graph, typename Vertex>
std::stack<Vertex> KnownMapPathFinder::createVertexStackFromBFS(const Graph& graph, const Vertex source, const Vertex destination) {
    std::stack<Vertex> route;
    std::vector<Vertex> predecessors(num_vertices(graph), -1);
    // To improve the "generality" of this function we would need to find a better value than -1
    // to initialize the predecessor vector to, we get away with this here because our graph vertex descriptor
    // is int, anything that doesn't have a cast to int will cause errors.

    breadth_first_search(graph, vertex(source, graph),
        visitor(
            make_bfs_visitor(
                record_predecessors(
                    &predecessors[0],
                    on_tree_edge()
                )
            )
        )
    );

    if(predecessors[destination] == -1) {
        return std::stack<int>(); // Empty stack means no valid path was found or the source is the same as the destination.
    }

    for(int vertex = destination; vertex != -1; vertex = predecessors[vertex]){
        route.push(vertex);
    }

    return route;
}