#pragma once

#include "path_finder_manager.hpp"
#include <boost/graph/adjacency_list.hpp>

typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;

class UnknownMapPathFinder : public PathFinderManager {
  public:
    UnknownMapPathFinder(uint8_t heightDeltaThreshold);
    bool setGoal(std::pair<uint8_t, uint8_t>&) override;
    int getNextMove() override;
    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override;
  private:
    Graph _graph;
};