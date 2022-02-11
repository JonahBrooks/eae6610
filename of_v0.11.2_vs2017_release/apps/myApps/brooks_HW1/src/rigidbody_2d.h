#ifndef BROOKS_HW1_SRC_RIGIDBODY_2D_H
#define BROOKS_HW1_SRC_RIGIDBODY_2D_H

#include "ofVec2f.h"

namespace brooks_hw1 {
class Rigidbody2d {
 public:
  Rigidbody2d() : position_(0, 0), velocity_(0, 0), orientation_(0), rotation_(0) {};

	ofVec2f position_;
	ofVec2f velocity_;
  float orientation_;
	float rotation_;
};
} // namespace brooks_hw1

#endif // BROOKS_HW1_SRC_RIGIDBODY_2D_H
