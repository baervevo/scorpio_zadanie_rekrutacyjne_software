#pragma once

#include <array>
#include <stack>
#include <boost/graph/adjacency_list.hpp>
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/GetMap.h"
#include "autonomy_simulator/RoverMap.h"

using namespace boost;

class PathFinderManager {
  public:
    PathFinderManager(uint8_t heightDeltaThreshold);
    void setActivePose(std::array<int8_t, 3>&);
    
    virtual bool setGoal(std::pair<uint8_t, uint8_t>& goalData) {
      _goalX = goalData.first;
      _goalY = goalData.second;

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
};