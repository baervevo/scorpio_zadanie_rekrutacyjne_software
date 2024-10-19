#pragma once

#include <array>
#include <string>
#include <stack>
#include <std_msgs/UInt8.h>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "autonomy_simulator/RoverPose.h"
#include "path_finder.hpp"

inline int8_t determineMoveInstruction(int8_t roverPoseX, int8_t roverPoseY, int8_t roverPoseR, int8_t goalX, int8_t goalY) {
    // This function assumes that moving forward is desired. If not, we can further reduce the number of rotations by simply going backwards when convenient.

    int8_t deltaX = goalX - roverPoseX;
    int8_t deltaY = goalY - roverPoseY;

    int8_t desiredRoverPoseR = -1;

    // We arbitrarily prioritize the vertical direction.
    if (deltaY > 0) {
        desiredRoverPoseR = autonomy_simulator::RoverPose::ORIENTATION_NORTH;
    } else if (deltaY < 0) {
        desiredRoverPoseR = autonomy_simulator::RoverPose::ORIENTATION_SOUTH;
    } else if (deltaX > 0) {
        desiredRoverPoseR = autonomy_simulator::RoverPose::ORIENTATION_EAST;
    } else if (deltaX < 0) {
        desiredRoverPoseR = autonomy_simulator::RoverPose::ORIENTATION_WEST;
    } // Theoretically no else statement is needed, deltaX == 0 && deltaY == 0 implies we are at our target, in which case this function shouldn't execute.

    if(desiredRoverPoseR == -1) {
        return -1;
    }

    int8_t leftDelta = (desiredRoverPoseR - roverPoseR + 4) % 4;
    int8_t rightDelta = (roverPoseR - desiredRoverPoseR + 4) % 4;

    if(leftDelta == 0 || rightDelta == 0) {
        return 2;
    } else if (leftDelta > rightDelta) {
        return 1;
    } else if (rightDelta < leftDelta) {
        return 0;
    } else {
        // If steps are equal (we have to rotate 180 degrees) we arbitrarily rotate right
        return 1;
    }
}

template <typename T>
inline std::string vectorToString(std::vector<T>& vec) {
    std::stringstream ss;

    ss << "[";  
    for(int i = 0; i < vec.size(); i++) {
        if(i != 0) {
            ss << ",";
        }

        ss << vec[i];
    }
    ss << "]";

    return ss.str();
}

inline std::pair<int8_t, int8_t> mapDataIndexToCoordinates(int i, int8_t width) {
    return {i % width, std::floor(i / width)};
}

inline int coordinatesToMapDataIndex(int x, int y, int width) {
    return y * width + x;
}

inline std::vector<std::pair<uint8_t, uint8_t>> manhattanNeighbours(int8_t x, int8_t y) {
    return {
        {x - 1, y},
        {x + 1, y},
        {x, y + 1},
        {x, y - 1}
    };
}

inline void populateGraphBasedOnMap(Graph& graph, const std::vector<int8_t>& mapData,
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
inline std::stack<int> createVertexStackFromBFS(const Graph& graph, const Vertex source, const Vertex destination) {
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

// Returns a vector of the updates indices so we don't need to parse unnecessary data to update our graph.
inline std::vector<int> updateMapBasedOnSensorData(std::vector<int8_t>& map, const std::vector<int8_t>& sensorData,
        const uint8_t roverPoseX, const uint8_t roverPoseY, const uint8_t roverPoseR, const int mapWidth) {

    int sensorWidth;
    
    if(roverPoseR == autonomy_simulator::RoverPose::ORIENTATION_NORTH || roverPoseR == autonomy_simulator::RoverPose::ORIENTATION_SOUTH) {
        sensorWidth = 3;
    } else {
        sensorWidth = 2;
    }

    int bottomLeftX;
    int bottomLeftY;

    switch (roverPoseR) {
        case autonomy_simulator::RoverPose::ORIENTATION_NORTH:
            bottomLeftX = roverPoseX - 1;
            bottomLeftY = roverPoseY + 1;
            break;
        case autonomy_simulator::RoverPose::ORIENTATION_EAST:
            bottomLeftX = roverPoseX + 1;
            bottomLeftY = roverPoseY - 1;
            break;
        case autonomy_simulator::RoverPose::ORIENTATION_SOUTH:
            bottomLeftX = roverPoseX - 1;
            bottomLeftY = roverPoseY - 2;
            break;
        case autonomy_simulator::RoverPose::ORIENTATION_WEST:
            bottomLeftX = roverPoseX - 2;
            bottomLeftY = roverPoseY - 1;
            break;
        default:
            bottomLeftX = -1;
            bottomLeftY = -1;
            break;
    }

    std::vector<int> updatedIndices;

    for(int i = 0; i < 5 - sensorWidth; i++) {
        for(int j = 0; j < sensorWidth; j++) {
            int mapIndex = coordinatesToMapDataIndex(bottomLeftX + j, bottomLeftY + i, mapWidth);
            int sensorDataIndex = coordinatesToMapDataIndex(j, i, sensorWidth);

            if(map[mapIndex] == -1) {
                updatedIndices.push_back(mapIndex);
            }

            map[mapIndex] = sensorDataIndex;
        }
    }

    return updatedIndices;
}

inline void updateGraphFromSensorData(Graph& graph, const std::vector<int>& updatedIndices,
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