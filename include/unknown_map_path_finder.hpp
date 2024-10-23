#pragma once

#include "path_finder_manager.hpp"
#include "path_finder_utils.hpp"
#include <boost/graph/adjacency_list.hpp>

typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;

class UnknownMapPathFinder : public PathFinderManager {
  public:
    UnknownMapPathFinder(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight);
    bool setGoal(std::pair<uint8_t, uint8_t>&) override;
    int getNextMove() override;
    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override;
  private:
    Graph _graph;

    static void updateGraphFromSensorData(Graph& graph, const std::vector<int>& updatedIndices,
          const std::vector<int8_t>& mapData, const int mapWidth, const int mapHeight,
          const int heightDeltaThreshold);

    static void depthFirstPathFindingRec(Graph& graph, std::vector<bool>& visited,
            Vertex destination, Vertex current);

    static void depthFirstPathFinding(Graph& graph, Vertex destination, Vertex source,
            std::function<void(int)> collector);
};