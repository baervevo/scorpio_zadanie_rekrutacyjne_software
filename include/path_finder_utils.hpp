#pragma once

#include <array>
#include <string>
#include <stack>
#include <sstream>
#include <std_msgs/UInt8.h>
#include <iomanip>
#include <fstream>

#include "autonomy_simulator/RoverPose.h"

inline int determineMoveInstruction(int8_t roverPoseX, int8_t roverPoseY, int8_t roverPoseR, int8_t goalX, int8_t goalY) {
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

inline void printStringToFile(const std::string& filename, const std::string& content) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << content;
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}


inline std::string printMap(std::vector<int8_t> mapData, int8_t mapWidth) {
    std::stringstream ss;

    for(int i = 0; i < mapData.size(); i++) {
        if(i % mapWidth == 0) {
            ss << "\n";
        }

        ss << std::setw(3) << static_cast<int>(mapData[i]);
    }

    return ss.str();
}

inline std::string stackToString(std::stack<int> s) {
    std::ostringstream oss;

    // Transfer elements from stack to string
    while (!s.empty()) {
        oss << s.top();
        s.pop();
        if (!s.empty()) {
            oss << " "; // Add space between elements
        }
    }

    return oss.str(); // Return the constructed string
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

inline void transposeMatrix(std::vector<std::vector<int>>& matrix) {
    // Get the number of rows and columns
    int rows = matrix.size();
    int cols = matrix[0].size();

    // Create a new matrix for the transposed result
    std::vector<std::vector<int>> transposedMatrix(cols, std::vector<int>(rows));

    // Transpose the original matrix
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            transposedMatrix[j][i] = matrix[i][j];
        }
    }

    std::reverse(transposedMatrix.begin(), transposedMatrix.end());
    
    matrix = transposedMatrix;
}

// Returns a vector of the updates indices so we don't need to parse unnecessary data to update our graph.
inline std::vector<int> updateMapBasedOnSensorData(std::vector<int8_t>& map, const std::vector<int8_t>& sensorData,
        const uint8_t roverPoseX, const uint8_t roverPoseY, const uint8_t roverPoseR, const int mapWidth, const int mapHeight) {

    int sensorWidth;
    
    if(roverPoseR == autonomy_simulator::RoverPose::ORIENTATION_NORTH || roverPoseR == autonomy_simulator::RoverPose::ORIENTATION_SOUTH) {
        sensorWidth = 3;
    } else {
        sensorWidth = 2;
    }

    int bottomLeftX;
    int bottomLeftY;
    int rotations;

    std::vector<std::vector<int>> indexMatrix = {
        {0, 1, 2},
        {3, 4, 5},
    };

    switch (roverPoseR) {
        case autonomy_simulator::RoverPose::ORIENTATION_NORTH:
            rotations = 0;
            bottomLeftX = roverPoseX - 1;
            bottomLeftY = roverPoseY + 1;
            break;
        case autonomy_simulator::RoverPose::ORIENTATION_EAST:
            rotations = 1;
            bottomLeftX = roverPoseX + 1;
            bottomLeftY = roverPoseY - 1;
            break;
        case autonomy_simulator::RoverPose::ORIENTATION_SOUTH:
            rotations = 2;
            bottomLeftX = roverPoseX - 1;
            bottomLeftY = roverPoseY - 2;
            break;
        case autonomy_simulator::RoverPose::ORIENTATION_WEST:
            rotations = 3;
            bottomLeftX = roverPoseX - 2;
            bottomLeftY = roverPoseY - 1;
            break;
        default:
            rotations = -1;
            bottomLeftX = -1;
            bottomLeftY = -1;
            break;
    }

    for(int i = 0; i < rotations; i++) {
        transposeMatrix(indexMatrix);
    }

    std::vector<int> updatedIndices;

    for(int i = 0; i < 5 - sensorWidth; i++) {
        for(int j = 0; j < sensorWidth; j++) {  
            int x = bottomLeftX + j;
            int y = bottomLeftY + i;

            if(x >= 0 && x < mapWidth && y >= 0 && y < mapHeight) {
                int mapIndex = coordinatesToMapDataIndex(x, y, mapWidth);  

                if(map[mapIndex] == -1) {
                    updatedIndices.push_back(mapIndex);

                    int currentPosInd = coordinatesToMapDataIndex(roverPoseX, roverPoseY, mapWidth);
                    int sensorInd = indexMatrix[i][j];

                    map[mapIndex] = map[currentPosInd] + sensorData[sensorInd];
                }
            }
        }
    }

    return updatedIndices;
}

inline float euclideanDistance(int source, int destination, int8_t mapWidth) {
    const auto& [x1, y1] = mapDataIndexToCoordinates(source, mapWidth);
    const auto& [x2, y2] = mapDataIndexToCoordinates(destination, mapWidth);

    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}