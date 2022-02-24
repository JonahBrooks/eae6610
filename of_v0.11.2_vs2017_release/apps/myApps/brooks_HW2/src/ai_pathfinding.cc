#include "ai_pathfinding.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <stack>
#include <vector>

namespace brooks_hw2 {

std::vector<Edge> AiPathfinding::Search(int start, int goal, const Graph& graph, HeuristicType heuristic) {
	std::vector<Edge> path;
	std::stack<Edge> reverse_path;
	std::priority_queue<NodeRecord, std::vector<NodeRecord>, NodeRecord> fringe;
	
	std::map<int, Edge> overall_cheapest_edge_into_node;

	std::list<NodeRecord> open_list;
	std::list<NodeRecord> closed_list;

	int (*h)(int, int, const Graph&) = &DijkstrasHeuristic;
	
	switch (heuristic) {
		case HeuristicType::kGuessMinimumEdgeWeight: {
			h = &GuessMinimumEdgeWeightHeuristic;
			break;
		}
		case HeuristicType::kGuess1: {
			h = &Guess1Heuristic;
		}
	}

	fringe.push(NodeRecord(start, Edge(), 0, 0 + h(start, goal, graph)));
	NodeRecord current = fringe.top();
	open_list.push_back(current);
	bool goal_found = false;
	while (fringe.size() > 0) {
		current = fringe.top();
		fringe.pop();
		
		if (current.node_id_ == goal) {
			goal_found = true;
			break;
		}
		std::vector<std::pair<int,int>> children = graph.GetAdjacentNodes(current.node_id_);
		for (std::pair<int, int> child : children) {
			int end_node = child.first; // Get the dest node
			int end_node_cost = current.cost_so_far_ + child.second; // Add the cost so far to the distance from source to dest
			int end_node_heuristic = 0;
			NodeRecord end_node_record;
			std::list<NodeRecord>::iterator end_node_record_iterator = std::find(closed_list.begin(), closed_list.end(), end_node);
			if (end_node_record_iterator != closed_list.end()) { // closed_list contains child
				end_node_record = *end_node_record_iterator;
				if (end_node_record.cost_so_far_ <= end_node_cost) {
					continue;
				}
				closed_list.remove(end_node_record);
				end_node_heuristic = end_node_record.estimated_total_cost_ - end_node_record.cost_so_far_;
			}
			else if ((end_node_record_iterator = std::find(open_list.begin(), open_list.end(), end_node)) != open_list.end()){
				// open_list contains child
				end_node_record = *end_node_record_iterator;
				if (end_node_record.cost_so_far_ <= end_node_cost) {
					continue;
				}
				end_node_heuristic = end_node_record.estimated_total_cost_ - end_node_record.cost_so_far_;
			}
			else {
				// Neither list contains child
				end_node_record = NodeRecord(end_node, Edge(current.node_id_, end_node, child.second), end_node_cost, INT_MAX);
				end_node_heuristic = h(end_node, goal, graph);
			}

			end_node_record.node_id_ = end_node;
			end_node_record.cost_so_far_ = end_node_cost;
			end_node_record.connection_ = Edge(current.node_id_, end_node, child.second);
			end_node_record.estimated_total_cost_ = end_node_cost + end_node_heuristic;

			overall_cheapest_edge_into_node[end_node] = end_node_record.connection_;

			if (std::find(open_list.begin(), open_list.end(), end_node) == open_list.end()) { // If not in open_list
				open_list.push_back(end_node_record);
				fringe.push(end_node_record);
			}
		}
		open_list.remove(current);
		closed_list.push_back(current);
	}
	if (current.node_id_ != goal) {
		goal_found = false;
	}

	if (goal_found) {
		std::cout << "Goal found along path: " << std::endl;
		int current_node_in_path = goal;
		while (current_node_in_path != start) {
			reverse_path.push(overall_cheapest_edge_into_node[current_node_in_path]);
			current_node_in_path = overall_cheapest_edge_into_node[current_node_in_path].source_;
		}
		reverse_path.push(Edge(start,start,0));
		while (!reverse_path.empty()) {
			path.push_back(reverse_path.top());
			std::cout << reverse_path.top().dest_ + 1 << ", ";
			reverse_path.pop();
		}
	}

	return path;
}

int AiPathfinding::DijkstrasHeuristic(int start, int goal, const Graph& graph) {
	return 0;
}

int AiPathfinding::Guess1Heuristic(int start, int goal, const Graph& graph) {
	return 1;
}

int AiPathfinding::GuessMinimumEdgeWeightHeuristic(int start, int goal, const Graph& graph) {
	return graph.GetSmallestEdgeWeight();
}

} // namespace brooks_hw2