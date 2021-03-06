#ifndef BROOKS_HW3_SRC_HW3_APP_H
#define BROOKS_HW3_SRC_HW3_APP_H

#include <queue>
#include <vector>

#include "ai_agent.h"
#include "graph.h"
#include "ofMain.h"

namespace brooks_hw3 {

// Note that the inherited member functions of this class do not adhere to
// Google's style guide
class Hw3App : public ofBaseApp {
 public:
  // Inherited member functions
  void setup();
  void update();
  void draw();

  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mousePressed(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void mouseEntered(int x, int y);
  void mouseExited(int x, int y);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
  // End inherited member functions
 private:
  void RunHeuristicAnalysis();
  void MakeWallAndEditEdges(size_t x, size_t y, size_t width, size_t height);
  ofVec2f GridToWorld(size_t x, size_t y);
  ofVec2f GridToWorld(size_t i);
  size_t WorldToGrid(ofVec2f location);

  Graph slc_graph_;
  Graph nyc_graph_;
  Graph indoor_graph_;

  ofRectangle heuristic_analysis_button_;

  ofVec2f grid_offset_;
  size_t grid_width_;
  size_t grid_height_;
  size_t grid_square_world_width_;
  size_t grid_square_world_height_;
  size_t grid_number_of_nodes_;

  std::vector<ofRectangle> walls_;
  std::vector<ofVec2f> grid_;
  std::vector<Edge> edges_;

  ofVec2f click_location_;

  AiAgent boid_;
};

}  // namespace brooks_hw3

#endif  // BROOKS_HW3_SRC_HW3_APP_H
