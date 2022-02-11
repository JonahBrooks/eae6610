#ifndef BROOKS_HW1_SRC_DYNAMIC_STEERING_OUTPUT_H
#define BROOKS_HW1_SRC_DYNAMIC_STEERING_OUTPUT_H

#include "ofVec2f.h"

namespace brooks_hw1 {
struct DynamicSteeringOutput {
 public:
  DynamicSteeringOutput() : linear_acceleration(0, 0), rotational_acceleration(0){};

  DynamicSteeringOutput operator+(const DynamicSteeringOutput& rhs);
  DynamicSteeringOutput operator-(const DynamicSteeringOutput& rhs);
  DynamicSteeringOutput operator*(float rhs);

  ofVec2f linear_acceleration;
  float rotational_acceleration;
};
} // namespace brooks_hw1

#endif // BROOKS_HW1_SRC_DYNAMIC_STEERING_OUTPUT_H
