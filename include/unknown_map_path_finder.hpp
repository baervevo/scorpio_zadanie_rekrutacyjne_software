#pragma once

#include "path_finder_manager.hpp"
#include "path_finder_utils.hpp"
#include <boost/graph/adjacency_list.hpp>

#include <vector>
#include <stack>

typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>> Graph;
typedef graph_traits<Graph>::vertex_descriptor	 Vertex;
typedef graph_traits<Graph>::adjacency_iterator AdjacencyIterator;

class UnknownMapPathFinder : public PathFinderManager {
  public:
    UnknownMapPathFinder(uint8_t heightDeltaThreshold, uint8_t gridWidth, uint8_t gridHeight);
    bool setGoal(std::pair<uint8_t, uint8_t>&) override;
    int getNextMove() override;
    void updateMapData(const autonomy_simulator::RoverMap::ConstPtr&) override;

	~UnknownMapPathFinder() override {
		delete _visited;
		delete _dfsStack;
		delete _activeRoute;
	}
  private:
    Graph _graph;

    std::vector<bool>* _visited;
    std::stack<Vertex>* _dfsStack;
	Vertex _nextVertex;

	std::stack<int>* _activeRoute;

	Vertex DFS();

    static void updateGraphFromSensorData(Graph& graph, const std::vector<int>& updatedIndices,
          const std::vector<int8_t>& mapData, const int mapWidth, const int mapHeight,
          const int heightDeltaThreshold);
};