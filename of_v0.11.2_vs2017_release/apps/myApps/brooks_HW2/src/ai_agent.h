#ifndef BROOKS_HW2_SRC_AI_AGENT_H
#define BROOKS_HW2_SRC_AI_AGENT_H

#include <deque>

#include "ofColor.h"
#include "ofVec2f.h"

#include "rigidbody_2d.h"
#include "dynamic_steering_output.h"
#include "kinematic_steering_output.h"

namespace brooks_hw2 {
class AiAgent {
 public:
  AiAgent() : rigidbody_(), color_(), radius_(10.0f),
    arrive_target_radius_(20.0f), arrive_slow_radius_(100.0f),
    top_speed_(100), drag_(0.99f), rotational_drag_(0.99f),
    separation_watchdog_timer_(0), flocking_mass_(1),
    time_per_breadcrumb_(0.5f), number_of_breadcrumbs_(20),
    breadcrumbs_(), time_since_last_breadcrumb_(0){};

  bool operator==(const AiAgent& rhs);
  bool operator!=(const AiAgent& rhs);

  void Update(float dt, DynamicSteeringOutput steering_output);
  void UpdateKinematic(float dt, KinematicSteeringOutput steering_output);
  void DrawAsBoid();
  void DrawAsPoint(bool draw_debug_circles = false);
  void MoveOffScreen();

  Rigidbody2d rigidbody_;
  ofColor color_;
  float radius_;
  float arrive_target_radius_;
  float arrive_slow_radius_;
  float top_speed_;
  float drag_;
  float rotational_drag_;
  float separation_watchdog_timer_;
  float flocking_mass_;
  float time_per_breadcrumb_;
  unsigned int number_of_breadcrumbs_;

 private:
  void UpdateBreadcrumbs();

  std::deque<ofVec2f> breadcrumbs_;
  float time_since_last_breadcrumb_;
};
} // namespace brooks_hw2

#endif // BROOKS_HW2_SRC_AI_AGENT_H