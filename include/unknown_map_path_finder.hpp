#pragma once

#include "path_finder_manager.hpp"

class UnknownMapPathFinder : public PathFinderManager {
  public:
    UnknownMapPathFinder(uint8_t heightDeltaThreshold);
    int getNextMove() override;
    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override;
};