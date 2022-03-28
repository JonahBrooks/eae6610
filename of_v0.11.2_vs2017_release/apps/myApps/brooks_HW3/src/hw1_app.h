#ifndef BROOKS_HW3_SRC_HW1_APP_H
#define BROOKS_HW3_SRC_HW1_APP_H

#include <vector>

#include "ofMain.h"

#include "ai_agent.h"
#include "rigidbody_2d.h"

namespace brooks_hw3 {

// Note that the inherited member functions of this class do not adhere to Google's style guide
class Hw1App : public ofBaseApp {
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
  enum class DemoType { kKinematic = 0, kSeek, kSeekArrive, kWander, kWander2, kFlocking };
   
  void SetupKinematicDemo();
  void SetupSeekDemo();
  void SetupSeekArriveDemo();
  void SetupWanderDemo();
  void SetupWander2Demo();
  void SetupFlockingDemo();

  void UpdateKinematicDemo(float dt);
  void UpdateSeekDemo(float dt);
  void UpdateSeekArriveDemo(float dt);
  void UpdateWanderDemo(float dt);
  void UpdateWander2Demo(float dt);
  void UpdateFlockingDemo(float dt);

  std::vector<AiAgent> agents_;
  float last_time_step_;
  ofRectangle kinematic_demo_button_;
  ofRectangle seek_demo_button_;
  ofRectangle seek_arrive_demo_button_;
  ofRectangle wander_demo_button_;
  ofRectangle wander2_demo_button_;
  ofRectangle flocking_demo_button_;
  DemoType demo_type_;
  std::vector<Rigidbody2d> kinematic_target_points_;
  unsigned int current_kinematic_target_index_;
  AiAgent seek_target_point_;
  AiAgent wander_target_point_;
  float last_wander_target_angle_;
  float number_of_flock_followers_;
  AiAgent flocking_centroid_;
};

} // namespace brooks_hw3

#endif // BROOKS_HW3_SRC_HW1_APP_H

