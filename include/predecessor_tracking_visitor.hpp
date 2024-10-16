#pragma once

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <vector>

class predecessorTrackingVisitor : public boost::default_bfs_visitor {
  private:
    std::vector<int>& predecessors;

  public:
    predecessorTrackingVisitor(std::vector<int>& predecessors) :
        predecessors(predecessors) {
    }

    template <typename Vertex, typename Graph>
    void discover_vertex(Vertex vertex, const Graph& graph) {
    }

    template <typename Edge, typename Graph>
    void examine_edge(Edge edge, const Graph& graph) {
        auto sourceVertex = source(edge, graph);
        auto targetVertex = target(edge, graph);

        if(predecessors[targetVertex] == -1) {
            predecessors[targetVertex] = sourceVertex;
        }
    }
};