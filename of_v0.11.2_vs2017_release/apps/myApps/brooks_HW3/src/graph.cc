#include "graph.h"

#include <vector>

namespace brooks_hw3 {

	Graph::Graph(const std::vector<Edge>& edges, int number_of_nodes, int smallest_edge_weight) {
	smallest_edge_weight_ = smallest_edge_weight;
	number_of_nodes_ = number_of_nodes;

	adjacency_dests_and_weights_.resize(number_of_nodes, {});
	for (const Edge& edge : edges) {
		adjacency_dests_and_weights_[edge.source_].push_back(std::pair<int, int>(edge.dest_, edge.weight_));
	}
}

void Graph::Initialize(const std::vector<Edge>& edges, int number_of_nodes, int smallest_edge_weight) {
	smallest_edge_weight_ = smallest_edge_weight;
	number_of_nodes_ = number_of_nodes;

	adjacency_dests_and_weights_.clear();
	adjacency_dests_and_weights_.resize(number_of_nodes, {});
	for (const Edge& edge : edges) {
		adjacency_dests_and_weights_[edge.source_].push_back(std::pair<int, int>(edge.dest_, edge.weight_));
	}
}

std::vector<std::pair<int, int>> Graph::GetAdjacentNodes(int node) const {
	return adjacency_dests_and_weights_[node];
}

int Graph::GetNumberOfNodes() const {
	return number_of_nodes_;
}

int Graph::GetSmallestEdgeWeight() const {
	return smallest_edge_weight_;
}

} // namespace brooks_hw3