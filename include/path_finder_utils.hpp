#pragma once

#include <utility>
#include <array>
#include <string>
#include <stack>
#include <std_msgs/UInt8.h>
#include "autonomy_simulator/RoverPose.h"
#include "path_finder.hpp"
#include "autonomy_simulator.hpp"
#include <stack>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adjacency_list.hpp>

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
    return {std::floor(i / width), i % width};
}

inline int coordinatesToMapDataIndex(int x, int y, int width) {
    return y * width + x;
}

inline void populateGraphBasedOnMap(Graph& graph, const std::vector<int8_t>& mapData,
        const int heightDeltaThreshold, const int8_t mapWidth, const int8_t mapHeight) {

    for(int i = 0; i < GRID_SIZE; i++) {
        for(int j = 0; j < GRID_SIZE; j++) {
            int currentIndex = coordinatesToMapDataIndex(i, j, mapWidth);

            std::vector<std::pair<uint8_t, uint8_t>> potentialNeighbours = {
                {i - 1, j},
                {i + 1, j},
                {i, j + 1},
                {i, j - 1}
            };

            for(const auto& [ni, nj] : potentialNeighbours) {
                if(ni >= 0 && ni < GRID_SIZE && nj >= 0 && nj < GRID_SIZE) {
                    int neighbourIndex = coordinatesToMapDataIndex(ni, nj, mapWidth);
                    int heightDelta = std::abs(mapData[currentIndex] - mapData[neighbourIndex]);

                    if(heightDelta <= heightDeltaThreshold) {
                        boost::add_edge(currentIndex, neighbourIndex, graph);
                    }
                }
            }
        }
    }
}

inline std::stack<int> createCoordinateStack(Graph& graph, uint8_t sourceX, uint8_t sourceY,
        uint8_t destX, uint8_t destY, uint8_t gridWidth) {
    
    std::stack<int> route;
    std::vector<int> predecessors(num_vertices(graph), -1);

    int sourceIndex = coordinatesToMapDataIndex(sourceX, sourceY, gridWidth);
    int destIndex = coordinatesToMapDataIndex(destX, destY, gridWidth);

    breadth_first_search(graph, vertex(sourceIndex, graph), predecessor_map(make_iterator_property_map(
        predecessors.begin(), get(vertex_index, graph)
    )));

    if(predecessors[destIndex] == -1) {
        return std::stack<int>(); // Empty stack means no valid path was found or the source is the same as the destination.
    }

    for(int vertex = destIndex; vertex != -1; vertex = predecessors[vertex]){
        route.push(vertex);
    }

    return route;
}