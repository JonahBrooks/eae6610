#ifndef BROOKS_HW3_SRC_GRAPH_OUTPUT_H
#define BROOKS_HW3_SRC_GRAPH_OUTPUT_H

#include <vector>

namespace brooks_hw3 {
class Edge {
 public:
	 Edge() : source_(0), dest_(0), weight_(0) {};
	 Edge(int source, int dest, int weight) : source_(source), dest_(dest), weight_(weight) {};
	 bool operator==(const Edge& rhs) {
		 return source_ == rhs.source_ && dest_ == rhs.dest_;
	 }
	 int source_;
	 int dest_;
	 int weight_;
};

class Graph {
 public:
	Graph() : smallest_edge_weight_(INT_MAX), number_of_nodes_(0) {};
	Graph(const std::vector<Edge>& edges, int number_of_nodes, int smallest_edge_weigth);
	void Initialize(const std::vector<Edge>& edges, int number_of_nodes, int smallest_edge_weight);
	std::vector<std::pair<int, int>> GetAdjacentNodes(int node) const;
	int GetNumberOfNodes() const;
	int GetSmallestEdgeWeight() const;

 private:
	int smallest_edge_weight_;
	int number_of_nodes_;
	std::vector < std::vector<std::pair<int, int>>> adjacency_dests_and_weights_;
};
} // namespace brooks_hw3

#endif // BROOKS_HW3_SRC_GRAPH_OUTPUT_H
