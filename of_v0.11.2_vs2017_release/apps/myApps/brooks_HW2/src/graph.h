#ifndef BROOKS_HW2_SRC_GRAPH_OUTPUT_H
#define BROOKS_HW2_SRC_GRAPH_OUTPUT_H

#include <vector>

namespace brooks_hw2 {
class Edge {
 public:
	 Edge() : source_(0), dest_(0), weight_(0) {};
	 Edge(int source, int dest, int weight) : source_(source), dest_(dest), weight_(weight) {};
	 int source_;
	 int dest_;
	 int weight_;
};

class Graph {
 public:
	Graph() : number_of_nodes_(0) {};
	Graph(const std::vector<Edge>& edges, int number_of_nodes);
	void Initialize(const std::vector<Edge>& edges, int number_of_nodes);
	std::vector<std::pair<int, int>> GetAdjacentNodes(int node) const;
	int GetNumberOfNodes() const;

 private:
	int number_of_nodes_;
	std::vector < std::vector<std::pair<int, int>>> adjacency_dests_and_weights_;
};
} // namespace brooks_hw2

#endif // BROOKS_HW2_SRC_GRAPH_OUTPUT_H
