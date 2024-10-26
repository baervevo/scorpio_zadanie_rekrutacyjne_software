#pragma once

#include <array>
#include <stack>
#include <ros/ros.h>
#include "path_finder_utils.hpp"
#include <boost/graph/adjacency_list.hpp>
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/GetMap.h"
#include "autonomy_simulator/RoverMap.h"

using namespace boost;

class PathFinderManager {
  public:
    PathFinderManager(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight);
    void setActivePose(std::array<int8_t, 3>&);

	std::vector<int8_t>& getMapData() {
		return _map;
	};
    
    virtual bool setGoal(std::pair<uint8_t, uint8_t>& goalData) {
        int8_t goalX = goalData.first;
        int8_t goalY = goalData.second;

        if(goalX < 0 || goalX >= _gridWidth || goalY < 0 || goalY >= _gridHeight) {
			ROS_ERROR("Coordinates don't belong to map.");
            return false;
        }

        _goalX = goalX;
        _goalY = goalY;

		//ROS_INFO("TARGET HEIGHT: %d", _map[coordinatesToMapDataIndex(_goalX, _goalY, _gridWidth)]);

        return true;
    };
    
    virtual void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) = 0;
    virtual int getNextMove() = 0;
    virtual ~PathFinderManager() = default;

  protected:
    int8_t _roverPoseX;
    int8_t _roverPoseY;
    autonomy_simulator::RoverPose::_orientation_type _roverPoseR;

    uint8_t _goalX;
    uint8_t _goalY;

    uint8_t _heightDeltaThreshold;
    std::vector<int8_t> _map;

	uint8_t _gridWidth;
	uint8_t _gridHeight;
};