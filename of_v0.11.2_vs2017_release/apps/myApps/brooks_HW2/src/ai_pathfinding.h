#ifndef BROOKS_HW2_SRC_AI_PATHFINDING_H
#define BROOKS_HW2_SRC_AI_PATHFINDING_H

#include <vector>

#include "graph.h"

namespace brooks_hw2 {

enum class HeuristicType { kDijkstras = 0, kConstantGuess};

//class Connection {
// public:
//  Connection() : source_(NodeRecord()), dest_(NodeRecord()), weight_(0) {}
//  Connection(NodeRecord source, NodeRecord dest, int weight) : source_(source), dest_(dest), weight_(weight) {}
//  NodeRecord source_;
//  NodeRecord dest_;
//  int weight_;
//};

class NodeRecord {
 public:
   NodeRecord() : node_id_(0), connection_(Edge()), cost_so_far_(INT_MAX), estimated_total_cost_(INT_MAX) {}
   NodeRecord(int node_id, Edge connection, int cost_so_far, int estimated_total_cost) : 
     node_id_(node_id), connection_(connection), cost_so_far_(cost_so_far), 
     estimated_total_cost_(estimated_total_cost) {}

  int node_id_;
  Edge connection_;
  int cost_so_far_;
  int estimated_total_cost_;

  constexpr bool operator()(const NodeRecord& lhs, const NodeRecord& rhs) const noexcept {
    return lhs.estimated_total_cost_ > rhs.estimated_total_cost_;
  }

  bool operator==(const NodeRecord& rhs) const {
    return node_id_ == rhs.node_id_;
  }

  bool operator==(int id) const {
    return node_id_ == id;
  }
};

class AiPathfinding {
 public:
   static std::vector<Edge> Search(int start, int goal, const Graph& graph, HeuristicType heuristic = HeuristicType::kDijkstras);
 private:
   AiPathfinding() = delete;
  ~AiPathfinding() = delete;

  static int DijkstrasHeuristic(int start, int goal, const Graph& graph);
  static int ConstantGuessHeuristic(int start, int goal, const Graph& graph);
};

}

#endif // BROOKS_HW2_SRC_AI_PATHFINDING_H