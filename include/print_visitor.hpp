#pragma once

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>

using namespace boost;

class printVisitor : public default_bfs_visitor {
  public:
    template <typename Vertex, typename Graph>
    void discover_vertex(Vertex v, const Graph& g) const {
        std::cout << "Discovered vertex: " << v << std::endl;
    }
};