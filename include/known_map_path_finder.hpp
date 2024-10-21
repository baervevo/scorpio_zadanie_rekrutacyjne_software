#pragma once

#include "path_finder_manager.hpp"
#include <boost/graph/adjacency_list.hpp>

typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;

class KnownMapPathFinder : public PathFinderManager {
  public:
    KnownMapPathFinder(uint8_t heightDeltaThreshold, std::vector<int8_t>&);
    int getNextMove() override;
    bool setGoal(std::pair<uint8_t, uint8_t>&) override;

    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override {
      // do nothing.
    };
  private:
    std::stack<int> _activeRoute;
    Graph _graph;
};