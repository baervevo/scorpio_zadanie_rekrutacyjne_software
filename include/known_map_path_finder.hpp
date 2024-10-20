#pragma once

#include "path_finder_manager.hpp"

class KnownMapPathFinder : public PathFinderManager {
  public:
    KnownMapPathFinder(uint8_t heightDeltaThreshold, std::vector<int8_t>&);
    int getNextMove() override;
    bool setGoal(std::pair<uint8_t, uint8_t>&) override;

    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override {
      // do nothing.
    };
};