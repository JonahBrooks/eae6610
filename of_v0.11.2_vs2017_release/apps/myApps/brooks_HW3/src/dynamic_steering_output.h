#ifndef BROOKS_HW3_SRC_DYNAMIC_STEERING_OUTPUT_H
#define BROOKS_HW3_SRC_DYNAMIC_STEERING_OUTPUT_H

#include "ofVec2f.h"

namespace brooks_hw3 {
struct DynamicSteeringOutput {
 public:
  DynamicSteeringOutput() : linear_acceleration(0, 0), rotational_acceleration(0){};

  DynamicSteeringOutput operator+(const DynamicSteeringOutput& rhs);
  DynamicSteeringOutput operator-(const DynamicSteeringOutput& rhs);
  DynamicSteeringOutput operator*(float rhs);

  ofVec2f linear_acceleration;
  float rotational_acceleration;
};
} // namespace brooks_hw3

#endif // BROOKS_HW3_SRC_DYNAMIC_STEERING_OUTPUT_H
