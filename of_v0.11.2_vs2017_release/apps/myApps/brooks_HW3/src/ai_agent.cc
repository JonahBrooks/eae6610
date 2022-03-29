
#include "ai_agent.h"

#include <deque>

#include "ofMathConstants.h"
#include "ofGraphics.h"
#include "ofMain.h"

#include "dynamic_steering_output.h"
#include "kinematic_steering_output.h"
#include "rigidbody_2d.h"

namespace brooks_hw3 {

bool AiAgent::operator==(const AiAgent& rhs) {
  return rigidbody_.position_ == rhs.rigidbody_.position_ &&
    rigidbody_.velocity_ == rhs.rigidbody_.velocity_ &&
    rigidbody_.orientation_ == rhs.rigidbody_.orientation_ &&
    rigidbody_.rotation_ == rhs.rigidbody_.rotation_;
}

bool AiAgent::operator!=(const AiAgent& rhs) {
  return rigidbody_.position_ != rhs.rigidbody_.position_ ||
         rigidbody_.velocity_ != rhs.rigidbody_.velocity_ ||
         rigidbody_.orientation_ != rhs.rigidbody_.orientation_ ||
         rigidbody_.rotation_ != rhs.rigidbody_.rotation_;
}

void AiAgent::Update(float dt, DynamicSteeringOutput steering_output) {
  rigidbody_.velocity_ += steering_output.linear_acceleration * dt;
  // Clamp velocity to top speed
  if (rigidbody_.velocity_.length() > top_speed_) {
    rigidbody_.velocity_.normalize();
    rigidbody_.velocity_ *= top_speed_;
  }
  // Implement drag
  rigidbody_.velocity_ -= rigidbody_.velocity_ * drag_ * dt;
  // Update position
  rigidbody_.position_ += rigidbody_.velocity_*dt;
  // Wrap the screen
  float screen_width = ofGetWidth();
  float screen_size_buffer = 2 * radius_; // Make sure the boid goes completely off screen before wrapping
  while (rigidbody_.position_.x >= screen_width + screen_size_buffer) {
    rigidbody_.position_.x -= (screen_width + 2 * screen_size_buffer);
  }
  while (rigidbody_.position_.x < -screen_size_buffer) {
    rigidbody_.position_.x += (screen_width + 2 * screen_size_buffer);
  }
  float screen_height = ofGetHeight();
  while (rigidbody_.position_.y >= screen_height + screen_size_buffer) {
    rigidbody_.position_.y -= (screen_height + 2 * screen_size_buffer);
  }
  while (rigidbody_.position_.y < -screen_size_buffer) {
    rigidbody_.position_.y += (screen_height + 2 * screen_size_buffer);
  }

  rigidbody_.rotation_ += steering_output.rotational_acceleration * dt;
  while (rigidbody_.rotation_ >= PI) {
    rigidbody_.rotation_ -= 2 * PI;
  }
  while (rigidbody_.rotation_ < -PI) {
    rigidbody_.rotation_ += 2 * PI;
  }

  // Implement rotational drag
  rigidbody_.rotation_ -= rigidbody_.rotation_ * rotational_drag_ * dt;

  rigidbody_.orientation_ += rigidbody_.rotation_ * dt;
  while (rigidbody_.orientation_ >= PI) {
    rigidbody_.orientation_ -= 2 * PI;
  }
  while (rigidbody_.orientation_ < -PI) {
    rigidbody_.orientation_ += 2 * PI;
  }

  time_since_last_breadcrumb_ += dt;
  if (time_since_last_breadcrumb_ >= time_per_breadcrumb_) {
    time_since_last_breadcrumb_ = 0;
    UpdateBreadcrumbs();
  }

  if (!points_to_travel_.empty())
  {
      float threshold_for_counting_at_point = 1;
      if (rigidbody_.position_.distance(points_to_travel_.front()) <=
          threshold_for_counting_at_point) {
          points_to_travel_.pop();
      }
  }

}

void AiAgent::UpdateKinematic(float dt, KinematicSteeringOutput steering_output) {
  rigidbody_.velocity_ = steering_output.linear_velocity;
  rigidbody_.position_ += rigidbody_.velocity_ * dt;

  rigidbody_.rotation_ = steering_output.rotational_velocity;
  while (rigidbody_.rotation_ >= 2 * PI) {
    rigidbody_.rotation_ -= 2 * PI;
  }
  while (rigidbody_.rotation_ < 0) {
    rigidbody_.rotation_ += 2 * PI;
  }

  rigidbody_.orientation_ += rigidbody_.rotation_ * dt;
  while (rigidbody_.orientation_ >= 2 * PI) {
    rigidbody_.orientation_ -= 2 * PI;
  }
  while (rigidbody_.orientation_ < 0) {
    rigidbody_.orientation_ += 2 * PI;
  }

  time_since_last_breadcrumb_ += dt;
  if (time_since_last_breadcrumb_ >= time_per_breadcrumb_) {
    time_since_last_breadcrumb_ = 0;
    UpdateBreadcrumbs();
  }
}

void AiAgent::DrawAsBoid(bool draw_debug_path) {
  // Draw the boid
  ofPushMatrix();
  ofSetColor(color_);
  ofDrawCircle(rigidbody_.position_, radius_);
  ofVec2f triangle_point_1 = ofVec2f(0, radius_);
  ofVec2f triangle_point_2 = ofVec2f(0, -radius_);
  ofVec2f triangle_point_3 = ofVec2f(2 * radius_, 0);
  ofTranslate(rigidbody_.position_);
  ofRotateRad(rigidbody_.orientation_);
  ofDrawTriangle(triangle_point_1, triangle_point_2, triangle_point_3);
  ofPopMatrix();

  // Draw breadcrumbs
  for (unsigned int i = 0; i < breadcrumbs_.size(); i++) {
    ofDrawCircle(breadcrumbs_[i], radius_ / 4.0f);
  }

  // Draw the debug path, if requested and present
  if (draw_debug_path && !points_to_travel_.empty()) {
      ofPushMatrix();
      ofSetColor(ofColor::yellow);
      std::queue<ofVec2f> points_to_draw = points_to_travel_;
      while (!points_to_draw.empty()) {
          ofDrawCircle(points_to_draw.front(), 4);
          points_to_draw.pop();
      }
      ofPopMatrix();
  }
}

void AiAgent::DrawAsPoint(bool draw_debug_circles) {
  ofPushMatrix();
  // Draw the debug circles, if draw_debug_circles is true
  if (draw_debug_circles) {
    ofSetColor(ofColor::blue);
    ofDrawCircle(rigidbody_.position_, arrive_slow_radius_);
    ofSetColor(ofColor::green);
    ofDrawCircle(rigidbody_.position_, arrive_target_radius_);
  }
  ofSetColor(color_);
  ofDrawCircle(rigidbody_.position_, radius_);
  ofPopMatrix();
}

void AiAgent::MoveOffScreen() { 
  rigidbody_.position_ = ofVec2f(-1000, -1000); 
}

void AiAgent::SetPathToFollow(std::queue<ofVec2f> path)
{
    points_to_travel_ = path;
}

bool AiAgent::HasPathToFollow()
{
    return !points_to_travel_.empty();
}

ofVec2f AiAgent::NextPointToFollow()
{
    if (HasPathToFollow())
    {
        return points_to_travel_.front();
    }
    else
    {
        return ofVec2f(-1, -1);
    }
}

void AiAgent::UpdateBreadcrumbs() {
  breadcrumbs_.push_back(ofVec2f(rigidbody_.position_));
  while (breadcrumbs_.size() > number_of_breadcrumbs_) {
    breadcrumbs_.pop_front();
  }
}
} // namespace brooks_hw3