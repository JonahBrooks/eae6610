#ifndef BROOKS_HW1_SRC_KINEMATIC_STEERING_OUTPUT_H
#define BROOKS_HW1_SRC_KINEMATIC_STEERING_OUTPUT_H

#include "ofVec2f.h"

namespace brooks_hw1 {
struct KinematicSteeringOutput {
 public:
  KinematicSteeringOutput() : linear_velocity(0, 0), rotational_velocity(0){};

  ofVec2f linear_velocity;
  float rotational_velocity;
};
}  // namespace brooks_hw1

#endif  // BROOKS_HW1_SRC_KINEMATIC_STEERING_OUTPUT_H
