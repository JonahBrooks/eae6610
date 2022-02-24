#include "hw2_app.h"

#include <string>
#include <sstream>
#include <vector>

#include "ofColor.h"
#include "ofUtils.h"

#include "ai_agent.h"
#include "ai_behaviors.h"

// Helper functions
namespace {

bool ParseGraph(const char* path, vector<vector<float>>& out_graph_to_fill) {
  char marker;
  char data_to_drop[5] = " sp ";
  int first_int;
  int second_int;
  int third_int;
  ofBuffer buffer = ofBufferFromFile(path);
  if (buffer.size() <= 0) {
    std::cout << "Failed to read from file at path " << path << std::endl;
    return false;
  }
  for (string line : buffer.getLines()) {
    stringstream stream(line);
    stream.get(marker);
    std::cout << marker;
    switch (marker) {
      case 'p': {
        stream.get(data_to_drop, 4); // Drop the " sp "
        std::cout << data_to_drop;
        stream >> first_int;
        if (stream.fail()) {
          return false;
        }
        stream.get(); // Drop the space
        stream >> second_int;
        if (stream.fail()) {
          return false;
        }
        std::cout << first_int << " " << second_int;
        break;
      }
    }
    std::cout << std::endl;
  }
  return true;
}
}

namespace brooks_hw2 {

//--------------------------------------------------------------
void Hw2App::setup() { 
  std::cout << ParseGraph("SLC.gr", slc_graph_) << std::endl;
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
