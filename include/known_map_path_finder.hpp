#pragma once

#include "path_finder_manager.hpp"
#include "path_finder_utils.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;

class KnownMapPathFinder : public PathFinderManager {
  public:
    KnownMapPathFinder(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight, std::vector<int8_t>&);
    int getNextMove() override;
    bool setGoal(std::pair<uint8_t, uint8_t>&) override;

    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override {
      // do nothing.
    };

    template <typename Graph, typename Vertex>
    static std::stack<Vertex> createVertexStackFromBFS(const Graph& graph, const Vertex source, const Vertex destination);
  private:
    std::stack<int> _activeRoute;
    Graph _graph;

    static void populateGraphBasedOnMap(Graph& graph, const std::vector<int8_t>& mapData,
            const int heightDeltaThreshold, const int8_t mapWidth, const int8_t mapHeight);
};