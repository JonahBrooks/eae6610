#include "hw2_app.h"

#include <string>
#include <sstream>
#include <vector>

#include "ofColor.h"
#include "ofUtils.h"

#include "ai_agent.h"
#include "ai_behaviors.h"
#include "graph.h"

// Helper functions
namespace {

bool ParseGraph(const char* path, brooks_hw2::Graph& out_graph_to_fill) {
  char marker = 'c';
  char data_to_drop[5] = " sp ";
  int number_of_nodes = 0;
  int number_of_edges = 0;
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
          current_edge++;
        }
        break;
      }
    }
    out_graph_to_fill.Initialize(edges, number_of_nodes);
  }
  return true;
}
}

namespace brooks_hw2 {

//--------------------------------------------------------------
void Hw2App::setup() { 
  ParseGraph("SLC.gr", slc_graph_);
  ParseGraph("NYC.gr", nyc_graph_);
}

//--------------------------------------------------------------
void Hw2App::update() {
  
}

//--------------------------------------------------------------
void Hw2App::draw() {

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

} // namespace brooks_hw2
