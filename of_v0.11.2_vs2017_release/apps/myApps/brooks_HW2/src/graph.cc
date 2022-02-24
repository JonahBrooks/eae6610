#include "graph.h"

#include <vector>

namespace brooks_hw2 {

Graph::Graph(const std::vector<Edge>& edges, int number_of_nodes) {
	number_of_nodes_ = number_of_nodes;

	adjacency_dests_and_weights_.resize(number_of_nodes, {});
	for (const Edge& edge : edges) {
		adjacency_dests_and_weights_[edge.source_].push_back(std::pair<int, int>(edge.dest_, edge.weight_));
	}
}

void Graph::Initialize(const std::vector<Edge>& edges, int number_of_nodes) {
	number_of_nodes_ = number_of_nodes;

	adjacency_dests_and_weights_.clear();
	adjacency_dests_and_weights_.resize(number_of_nodes, {});
	for (const Edge& edge : edges) {
		adjacency_dests_and_weights_[edge.source_].push_back(std::pair<int, int>(edge.dest_, edge.weight_));
	}
}

std::vector<std::pair<int, int>> Graph::GetAdjacentNodes(int node) {
	return adjacency_dests_and_weights_[node];
}

int Graph::GetNumberOfNodes() {
	return number_of_nodes_;
}

} // namespace brooks_hw2