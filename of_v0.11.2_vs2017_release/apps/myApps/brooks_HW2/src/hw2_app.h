#ifndef BROOKS_HW2_SRC_HW2_APP_H
#define BROOKS_HW2_SRC_HW2_APP_H

#include "ofMain.h"

namespace brooks_hw2 {

// Note that the inherited member functions of this class do not adhere to Google's style guide
class Hw2App : public ofBaseApp {
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
  
};

} // namespace brooks_hw2

#endif // BROOKS_HW2_SRC_HW2_APP_H

