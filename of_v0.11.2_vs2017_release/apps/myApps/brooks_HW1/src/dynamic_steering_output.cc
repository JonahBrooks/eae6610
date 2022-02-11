#include "dynamic_steering_output.h"

namespace brooks_hw1 {
DynamicSteeringOutput DynamicSteeringOutput::operator+(const DynamicSteeringOutput& rhs) {
  DynamicSteeringOutput output;
  output.linear_acceleration = this->linear_acceleration + rhs.linear_acceleration;
  output.rotational_acceleration = this->rotational_acceleration + rhs.rotational_acceleration;
  return output;
}

DynamicSteeringOutput DynamicSteeringOutput::operator-(const DynamicSteeringOutput& rhs) {
  DynamicSteeringOutput output;
  output.linear_acceleration = this->linear_acceleration - rhs.linear_acceleration;
  output.rotational_acceleration = this->rotational_acceleration - rhs.rotational_acceleration;
  return output;
}

DynamicSteeringOutput DynamicSteeringOutput::operator*(float rhs) {
  DynamicSteeringOutput output;
  output.linear_acceleration = this->linear_acceleration * rhs;
  output.rotational_acceleration = this->rotational_acceleration * rhs;
  return output;
}

}