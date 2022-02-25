#include "hw2_app.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "ofColor.h"
#include "ofUtils.h"

#include "ai_agent.h"
#include "ai_behaviors.h"
#include "ai_pathfinding.h"
#include "graph.h"

// Helper functions
namespace {

bool ParseGraph(const char* path, brooks_hw2::Graph& out_graph_to_fill) {
  char marker = 'c';
  char data_to_drop[5] = " sp ";
  int number_of_nodes = 0;
  int number_of_edges = 0;
  int smallest_edge_weight = INT_MAX;
  int current_edge = 0;
  int first_int = 0;
  int second_int = 0;
  int third_int = 0;
  std::vector<brooks_hw2::Edge> edges;
  ofBuffer buffer = ofBufferFromFile(path);
  if (buffer.size() <= 0) {
    std::cout << "Failed to read from file at path " << path << std::endl;
    return false;
  }
  for (string line : buffer.getLines()) {
    stringstream stream(line);
    stream.get(marker);
    switch (marker) {
      case 'p': { // We will always encounter a p before the first a
        stream.get(data_to_drop, 5); // Drop the " sp "
        stream >> number_of_nodes;
        if (stream.fail()) {
          return false;
        }
        stream >> number_of_edges;
        if (stream.fail()) {
          return false;
        }
        edges.resize(number_of_edges);
        break;
      }
      case 'a': {
        stream >> first_int;
        if (stream.fail()) {
          return false;
        }
        stream >> second_int;
        if (stream.fail()) {
          return false;
        }
        stream >> third_int;
        if (stream.fail()) {
          return false;
        }
        if (first_int <= edges.size() && second_int <= edges.size()) {
          first_int--;
          second_int--;
          edges[current_edge].source_ = first_int;
          edges[current_edge].dest_ = second_int;
          edges[current_edge].weight_ = third_int;
          if (third_int < smallest_edge_weight) {
            smallest_edge_weight = third_int;
          }
          current_edge++;
        }
        break;
      }
    }
    out_graph_to_fill.Initialize(edges, number_of_nodes, smallest_edge_weight);
  }
  return true;
}

brooks_hw2::DynamicSteeringOutput LookInDirectionOfTravel(brooks_hw2::AiAgent agent) {
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  float max_rotation = PI / 2;
  float align_target_angle = PI / 320;
  float align_slow_angle = PI / 2;
  float align_time_to_target = 1;
  return brooks_hw2::AiBehaviors::LookWhereYouAreGoing(
    agent.rigidbody_, brooks_hw2::Rigidbody2d(), max_rotation,
    align_slow_angle, align_target_angle,
    align_time_to_target);
}
}

namespace brooks_hw2 {

//--------------------------------------------------------------
void Hw2App::setup() { 
  heuristic_analysis_button_.set(10, 10, 200, 40);

  grid_offset_ = ofVec2f(50, 70);

  grid_width_ = 30;
  grid_height_ = 30;

  grid_square_world_width_ = (ofGetWidth() - grid_offset_.x)  / grid_width_;
  grid_square_world_height_ = (ofGetHeight() - grid_offset_.y) / grid_height_;

  grid_number_of_nodes_ = grid_width_ * grid_height_;

  for (int i = 0; i < grid_width_; i++) {
    for (int j = 0; j < grid_height_; j++) {
      grid_.push_back(ofVec2f(i * grid_square_world_width_ + grid_offset_.x, j * grid_square_world_height_ + grid_offset_.y));

      // Make down pointing edge
      if (j < grid_height_ - 1) {
        edges_.push_back(Edge(i + (j * grid_width_), i + (j * grid_width_) + grid_width_, 1));
      }
      // Make up pointing edge
      if ((j - 1) >= 0) {
        edges_.push_back(Edge(i + (j * grid_width_), i + (j * grid_width_) - grid_width_, 1));
      }
      // Make right pointing edge
      if (i < grid_width_ - 1) {
        edges_.push_back(Edge(i + (j * grid_width_), (i+1) + (j * grid_width_), 1));
      }
      // Make left pointing edge
      if ((i - 1) >= 0) {
        edges_.push_back(Edge(i + (j * grid_width_), (i-1) + (j * grid_width_), 1));
      }
    }
  }

  //walls_.push_back(ofRectangle(50, 100, 100, 40));

  // TODO: Remove edges blocked by walls

  indoor_graph_.Initialize(edges_, grid_number_of_nodes_, 1);

  boid_.color_ = ofColor(255, 0, 0);
  boid_.radius_ = 7;
  boid_.rigidbody_.position_.x = 50;
  boid_.rigidbody_.position_.y = ofGetHeight() - 50;
  boid_.number_of_breadcrumbs_ = 100;
  
  ofResetElapsedTimeCounter();
}

//--------------------------------------------------------------
void Hw2App::update() {
  float dt = ofGetElapsedTimef();;
  ofResetElapsedTimeCounter();
  if (points_to_travel_.empty()) {
    return;
  }
  float threshold_for_counting_at_point = 1;
  if (boid_.rigidbody_.position_.distance(points_to_travel_.front()) <= threshold_for_counting_at_point) {
    points_to_travel_.pop();
  }

  Rigidbody2d target;
  target.position_ = points_to_travel_.front();
  float max_linear_accel = 100;
  // Calculate the linear acceleration for the boid
  DynamicSteeringOutput steering_output = AiBehaviors::DynamicSeek(
    boid_.rigidbody_, target, max_linear_accel);
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  steering_output = steering_output + LookInDirectionOfTravel(boid_);
  boid_.Update(dt, steering_output);
}

//--------------------------------------------------------------
void Hw2App::draw() {
  
  ofSetColor(ofColor::blueSteel);
  for (ofVec2f grid_point : grid_) {
    ofDrawCircle(grid_point, 4);
  }

  ofSetColor(ofColor::whiteSmoke);
  for (Edge edge : edges_) {
    ofDrawLine(GridToWorld(edge.source_), GridToWorld(edge.dest_));
  }

  ofSetColor(ofColor::black);
  for (ofRectangle& wall : walls_) {
    ofDrawRectangle(wall);
  }


  boid_.DrawAsBoid();

  ofSetColor(ofColor::forestGreen);
  ofDrawCircle(click_location_, 5);

  ofSetColor(ofColor::gray);
  ofDrawRectangle(heuristic_analysis_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Log Heuristic Analysis", heuristic_analysis_button_.position + ofVec3f(10, 20, 0));
}

//--------------------------------------------------------------
void Hw2App::keyPressed(int key) {}

//--------------------------------------------------------------
void Hw2App::keyReleased(int key) {}

//--------------------------------------------------------------
void Hw2App::mouseMoved(int x, int y) {}

//--------------------------------------------------------------
void Hw2App::mouseDragged(int x, int y, int button) {}

//--------------------------------------------------------------
void Hw2App::mousePressed(int x, int y, int button) { 
  if (heuristic_analysis_button_.inside(x, y)) {
    RunHeuristicAnalysis();
  }
  else {
    click_location_ = ofVec2f(x, y);
    int nodes_visited = 0;
    std::vector<Edge> path = AiPathfinding::Search(WorldToGrid(boid_.rigidbody_.position_),
      WorldToGrid(click_location_), indoor_graph_, 
      nodes_visited, HeuristicType::kGuessMinimumEdgeWeight);
    for (Edge edge : path) {
      points_to_travel_.push(GridToWorld(edge.dest_));
    }
  }
}

//--------------------------------------------------------------
void Hw2App::mouseReleased(int x, int y, int button) {}

//--------------------------------------------------------------
void Hw2App::mouseEntered(int x, int y) {}

//--------------------------------------------------------------
void Hw2App::mouseExited(int x, int y) {}

//--------------------------------------------------------------
void Hw2App::windowResized(int w, int h) {}

//--------------------------------------------------------------
void Hw2App::gotMessage(ofMessage msg) {}

//--------------------------------------------------------------
void Hw2App::dragEvent(ofDragInfo dragInfo) {}

void Hw2App::RunHeuristicAnalysis() {
  int number_of_random_walkthroughs = 10;
  int nodes_visited = 0;
  int total_nodes_visited_across_walkthroughs = 0;
  float total_time_across_walkthroughs = 0;

  ParseGraph("SLC.gr", slc_graph_);

  // Find statistics for GuessMinimumEdgeWeight
  for (size_t i = 0; i < number_of_random_walkthroughs; i++) {
    nodes_visited = 0;
    ofResetElapsedTimeCounter();
    std::vector<Edge> slc_path = AiPathfinding::Search((int)ofRandom(0, slc_graph_.GetNumberOfNodes()),
      (int)ofRandom(0, slc_graph_.GetNumberOfNodes()),
      slc_graph_,
      nodes_visited,
      HeuristicType::kGuessMinimumEdgeWeight);
    total_time_across_walkthroughs += ofGetElapsedTimef();
    total_nodes_visited_across_walkthroughs += nodes_visited;
  }
  std::cout << "SLC graph with A* GuessMinimumEdgeWeightHeuristic:" << std::endl;
  std::cout << "\tWalkthroughs: " << number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Nodes Visited: " << total_nodes_visited_across_walkthroughs << std::endl;
  std::cout << "\tAverage Nodes Visited: " << (float)total_nodes_visited_across_walkthroughs / (float)number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Time Taken: " << total_time_across_walkthroughs << std::endl;
  std::cout << "\tAverage Time Taken: " << total_time_across_walkthroughs / number_of_random_walkthroughs << std::endl << std::endl;

  nodes_visited = 0;
  total_nodes_visited_across_walkthroughs = 0;
  total_time_across_walkthroughs = 0;

  // Find statistics for Guess1
  for (size_t i = 0; i < number_of_random_walkthroughs; i++) {
    nodes_visited = 0;
    ofResetElapsedTimeCounter();
    std::vector<Edge> slc_path = AiPathfinding::Search((int)ofRandom(0, slc_graph_.GetNumberOfNodes()),
      (int)ofRandom(0, slc_graph_.GetNumberOfNodes()),
      slc_graph_,
      nodes_visited,
      HeuristicType::kGuess1);
    total_time_across_walkthroughs += ofGetElapsedTimef();
    total_nodes_visited_across_walkthroughs += nodes_visited;
  }
  std::cout << "SLC graph with A* Guess1Heuristic:" << std::endl;
  std::cout << "\tWalkthroughs: " << number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Nodes Visited: " << total_nodes_visited_across_walkthroughs << std::endl;
  std::cout << "\tAverage Nodes Visited: " << (float)total_nodes_visited_across_walkthroughs / (float)number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Time Taken: " << total_time_across_walkthroughs << std::endl;
  std::cout << "\tAverage Time Taken: " << total_time_across_walkthroughs / number_of_random_walkthroughs << std::endl << std::endl;

  nodes_visited = 0;
  total_nodes_visited_across_walkthroughs = 0;
  total_time_across_walkthroughs = 0;

  // Find statistics for Dijkstra's
  for (size_t i = 0; i < number_of_random_walkthroughs; i++) {
    nodes_visited = 0;
    ofResetElapsedTimeCounter();
    std::vector<Edge> slc_path = AiPathfinding::Search((int)ofRandom(0, slc_graph_.GetNumberOfNodes()),
      (int)ofRandom(0, slc_graph_.GetNumberOfNodes()),
      slc_graph_,
      nodes_visited,
      HeuristicType::kDijkstras);
    total_time_across_walkthroughs += ofGetElapsedTimef();
    total_nodes_visited_across_walkthroughs += nodes_visited;
  }
  std::cout << "SLC graph with Dijkstra's:" << std::endl;
  std::cout << "\tWalkthroughs: " << number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Nodes Visited: " << total_nodes_visited_across_walkthroughs << std::endl;
  std::cout << "\tAverage Nodes Visited: " << (float)total_nodes_visited_across_walkthroughs / (float)number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Time Taken: " << total_time_across_walkthroughs << std::endl;
  std::cout << "\tAverage Time Taken: " << total_time_across_walkthroughs / number_of_random_walkthroughs << std::endl << std::endl;

  nodes_visited = 0;
  total_nodes_visited_across_walkthroughs = 0;
  total_time_across_walkthroughs = 0;

  ParseGraph("NYC.gr", nyc_graph_);
  // Find statistics for GuessMinimumEdgeWeight
  for (size_t i = 0; i < number_of_random_walkthroughs; i++) {
    nodes_visited = 0;
    ofResetElapsedTimeCounter();
    std::vector<Edge> nyc_path = AiPathfinding::Search((int)ofRandom(0, nyc_graph_.GetNumberOfNodes()),
      (int)ofRandom(0, nyc_graph_.GetNumberOfNodes()),
      nyc_graph_,
      nodes_visited,
      HeuristicType::kGuessMinimumEdgeWeight);
    total_time_across_walkthroughs += ofGetElapsedTimef();
    total_nodes_visited_across_walkthroughs += nodes_visited;
  }
  std::cout << "NYC graph with A* GuessMinimumEdgeWeightHeuristic:" << std::endl;
  std::cout << "\tWalkthroughs: " << number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Nodes Visited: " << total_nodes_visited_across_walkthroughs << std::endl;
  std::cout << "\tAverage Nodes Visited: " << (float)total_nodes_visited_across_walkthroughs / (float)number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Time Taken: " << total_time_across_walkthroughs << std::endl;
  std::cout << "\tAverage Time Taken: " << total_time_across_walkthroughs / number_of_random_walkthroughs << std::endl << std::endl;

  nodes_visited = 0;
  total_nodes_visited_across_walkthroughs = 0;
  total_time_across_walkthroughs = 0;

  // Find statistics for Guess1
  for (size_t i = 0; i < number_of_random_walkthroughs; i++) {
    nodes_visited = 0;
    ofResetElapsedTimeCounter();
    std::vector<Edge> nyc_path = AiPathfinding::Search((int)ofRandom(0, nyc_graph_.GetNumberOfNodes()),
      (int)ofRandom(0, nyc_graph_.GetNumberOfNodes()),
      nyc_graph_,
      nodes_visited,
      HeuristicType::kGuess1);
    total_time_across_walkthroughs += ofGetElapsedTimef();
    total_nodes_visited_across_walkthroughs += nodes_visited;
  }
  std::cout << "NYC graph with A* Guess1Heuristic:" << std::endl;
  std::cout << "\tWalkthroughs: " << number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Nodes Visited: " << total_nodes_visited_across_walkthroughs << std::endl;
  std::cout << "\tAverage Nodes Visited: " << (float)total_nodes_visited_across_walkthroughs / (float)number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Time Taken: " << total_time_across_walkthroughs << std::endl;
  std::cout << "\tAverage Time Taken: " << total_time_across_walkthroughs / number_of_random_walkthroughs << std::endl << std::endl;

  nodes_visited = 0;
  total_nodes_visited_across_walkthroughs = 0;
  total_time_across_walkthroughs = 0;

  // Find statistics for Dijkstra's
  for (size_t i = 0; i < number_of_random_walkthroughs; i++) {
    nodes_visited = 0;
    ofResetElapsedTimeCounter();
    std::vector<Edge> nyc_path = AiPathfinding::Search((int)ofRandom(0, nyc_graph_.GetNumberOfNodes()),
      (int)ofRandom(0, nyc_graph_.GetNumberOfNodes()),
      nyc_graph_,
      nodes_visited,
      HeuristicType::kDijkstras);
    total_time_across_walkthroughs += ofGetElapsedTimef();
    total_nodes_visited_across_walkthroughs += nodes_visited;
  }
  std::cout << "NYC graph with Dijkstra's:" << std::endl;
  std::cout << "\tWalkthroughs: " << number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Nodes Visited: " << total_nodes_visited_across_walkthroughs << std::endl;
  std::cout << "\tAverage Nodes Visited: " << (float)total_nodes_visited_across_walkthroughs / (float)number_of_random_walkthroughs << std::endl;
  std::cout << "\tTotal Time Taken: " << total_time_across_walkthroughs << std::endl;
  std::cout << "\tAverage Time Taken: " << total_time_across_walkthroughs / number_of_random_walkthroughs << std::endl << std::endl;

  nodes_visited = 0;
  total_nodes_visited_across_walkthroughs = 0;
  total_time_across_walkthroughs = 0;
}

ofVec2f Hw2App::GridToWorld(size_t x, size_t y) {
  return ofVec2f(x * grid_square_world_width_ + grid_offset_.x, y * grid_square_world_height_ + grid_offset_.y);
}

ofVec2f Hw2App::GridToWorld(size_t i) {
  size_t x = (i) % grid_width_;
  size_t y = (i) / grid_height_;
  return GridToWorld(x, y);
}

size_t Hw2App::WorldToGrid(ofVec2f location) {
  size_t x = (location.x - grid_offset_.x) / grid_square_world_width_;
  size_t y = (location.y - grid_offset_.y) / grid_square_world_height_;
  return x + (y * grid_width_);
}

} // namespace brooks_hw2
