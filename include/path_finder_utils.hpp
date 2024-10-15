#pragma once

#include <utility>
#include <array>
#include <std_msgs/UInt8.h>
#include "autonomy_simulator/RoverPose.h"

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