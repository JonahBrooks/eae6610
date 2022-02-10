#include "hw1_app.h"

#include <vector>

#include "ofColor.h"
#include "ofUtils.h"

#include "ai_agent.h"
#include "ai_behaviors.h"

// Helper functions
namespace {
brooks_hw1::DynamicSteeringOutput LookInDirectionOfTravel(brooks_hw1::AiAgent agent) {
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  float max_rotation = PI / 2;
  float align_target_angle = PI / 320;
  float align_slow_angle = PI / 2;
  float align_time_to_target = 1;
  return brooks_hw1::AiBehaviors::LookWhereYouAreGoing(
                            agent.rigidbody_, brooks_hw1::Rigidbody2d(), max_rotation,
                            align_slow_angle, align_target_angle,
                            align_time_to_target);
}
}

namespace brooks_hw1 {

//--------------------------------------------------------------
void Hw1App::setup() { 
  float button_x_padding = 20;
  float button_width = 150;
  float button_height = 40;
  kinematic_demo_button_.set(button_x_padding, 10, button_width, button_height);
  seek_demo_button_.set(button_x_padding * 2 + button_width * 1, 10, button_width, button_height);
  seek_arrive_demo_button_.set(button_x_padding * 3 + button_width * 2, 10, button_width, button_height);
  wander_demo_button_.set(button_x_padding * 4 + button_width * 3, 10, button_width, button_height);
  wander2_demo_button_.set(button_x_padding * 5 + button_width * 4, 10, button_width, button_height);
  flocking_demo_button_.set(button_x_padding * 6 + button_width * 5, 10, button_width, button_height);
	
  SetupKinematicDemo();
  last_time_step_ = ofGetElapsedTimef();

  wander_target_point_ = AiAgent();
  last_wander_target_angle_ = 0;

  number_of_flock_followers_ = 20;
  flocking_centroid_ = AiAgent();
}

//--------------------------------------------------------------
void Hw1App::update() {
  float dt = ofGetElapsedTimef() - last_time_step_;
  last_time_step_ = ofGetElapsedTimef();

  switch (demo_type_) { 
    case DemoType::kKinematic: {
      UpdateKinematicDemo(dt);
      break;
    }
    case DemoType::kSeek: {
      UpdateSeekDemo(dt);
      break;
    }
    case DemoType::kSeekArrive: {
      UpdateSeekArriveDemo(dt);
      break;
    }
    case DemoType::kWander: {
      UpdateWanderDemo(dt);
      break;
    }
    case DemoType::kWander2: {
      UpdateWander2Demo(dt);
      break;
    }
    case DemoType::kFlocking: {
      UpdateFlockingDemo(dt);
      break;
    }
  }
}

//--------------------------------------------------------------
void Hw1App::draw() {

  // Draw the targetting point for Seek, and the target point and slow radius and target radius for Seek/Arrive
  if (demo_type_ == DemoType::kSeek) {
    seek_target_point_.DrawAsPoint(false);
  } else if (demo_type_ == DemoType::kSeekArrive) {
    seek_target_point_.DrawAsPoint(true);
  } else if (demo_type_ == DemoType::kWander ||
             demo_type_ == DemoType::kWander2) {
    wander_target_point_.DrawAsPoint(false);
  } else if (demo_type_ == DemoType::kFlocking) {
    flocking_centroid_.DrawAsPoint(false);
  }

  // Draw the boids
  for (unsigned int i = 0; i < agents_.size(); i++) {
    agents_[i].DrawAsBoid();
  }

  // Draw buttons for selecting type of demo
  ofVec3f text_offset(20, 20,0);
  if (demo_type_ == DemoType::kKinematic) {
    ofSetColor(ofColor::white);
  } else {
    ofSetColor(ofColor::gray);
  }
  ofDrawRectangle(kinematic_demo_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Kinematic Demo", kinematic_demo_button_.position + text_offset);
  

  if (demo_type_ == DemoType::kSeek) {
    ofSetColor(ofColor::white);
  } else {
    ofSetColor(ofColor::gray);
  }
  ofDrawRectangle(seek_demo_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Pure Seek Demo", seek_demo_button_.position + text_offset);

  if (demo_type_ == DemoType::kSeekArrive) {
    ofSetColor(ofColor::white);
  } else {
    ofSetColor(ofColor::gray);
  }
  ofDrawRectangle(seek_arrive_demo_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Seek/Arrive Demo", seek_arrive_demo_button_.position + text_offset);

  if (demo_type_ == DemoType::kWander) {
    ofSetColor(ofColor::white);
  } else {
    ofSetColor(ofColor::gray);
  }
  ofDrawRectangle(wander_demo_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Wander Demo", wander_demo_button_.position + text_offset);
  
  if (demo_type_ == DemoType::kWander2) {
    ofSetColor(ofColor::white);
  } else {
    ofSetColor(ofColor::gray);
  }
  ofDrawRectangle(wander2_demo_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Wander2 Demo", wander2_demo_button_.position + text_offset);

  if (demo_type_ == DemoType::kFlocking) {
    ofSetColor(ofColor::white);
  } else {
    ofSetColor(ofColor::gray);
  }
  ofDrawRectangle(flocking_demo_button_);
  ofSetColor(ofColor::black);
  ofDrawBitmapString("Flocking Demo", flocking_demo_button_.position + text_offset);
}

//--------------------------------------------------------------
void Hw1App::keyPressed(int key) {}

//--------------------------------------------------------------
void Hw1App::keyReleased(int key) {}

//--------------------------------------------------------------
void Hw1App::mouseMoved(int x, int y) {}

//--------------------------------------------------------------
void Hw1App::mouseDragged(int x, int y, int button) {}

//--------------------------------------------------------------
void Hw1App::mousePressed(int x, int y, int button) { 
  if (kinematic_demo_button_.inside(x, y)) {
    demo_type_ = DemoType::kKinematic;
    SetupKinematicDemo();
  } else if (seek_demo_button_.inside(x, y)) {
    demo_type_ = DemoType::kSeek;
    SetupSeekDemo();
  } else if (seek_arrive_demo_button_.inside(x, y)) {
    demo_type_ = DemoType::kSeekArrive;
    SetupSeekArriveDemo();
  } else if (wander_demo_button_.inside(x, y)) {
    demo_type_ = DemoType::kWander;
    SetupWanderDemo();
  } else if (wander2_demo_button_.inside(x, y)) {
    demo_type_ = DemoType::kWander2;
    SetupWander2Demo();
  } else if (flocking_demo_button_.inside(x, y)) {
    demo_type_ = DemoType::kFlocking;
    SetupFlockingDemo();
  } else {
    // A non-button was clicked
    if (demo_type_ == DemoType::kSeek || demo_type_ == DemoType::kSeekArrive) {
      seek_target_point_.rigidbody_.position_ = ofVec2f(x, y);
    }
  }
}

//--------------------------------------------------------------
void Hw1App::mouseReleased(int x, int y, int button) {}

//--------------------------------------------------------------
void Hw1App::mouseEntered(int x, int y) {}

//--------------------------------------------------------------
void Hw1App::mouseExited(int x, int y) {}

//--------------------------------------------------------------
void Hw1App::windowResized(int w, int h) {}

//--------------------------------------------------------------
void Hw1App::gotMessage(ofMessage msg) {}

//--------------------------------------------------------------
void Hw1App::dragEvent(ofDragInfo dragInfo) {}

void Hw1App::SetupKinematicDemo() {
  seek_target_point_.MoveOffScreen();
  flocking_centroid_.MoveOffScreen();
  agents_.clear();
  agents_.push_back(AiAgent());
  agents_.back().color_ = ofColor(255, 0, 0);
  agents_.back().radius_ = 20;
  agents_.back().rigidbody_.position_.x = 50;
  agents_.back().rigidbody_.position_.y = ofGetHeight() - 50;
  agents_.back().number_of_breadcrumbs_ = 100;

  kinematic_target_points_.push_back(Rigidbody2d());
  kinematic_target_points_.push_back(Rigidbody2d());
  kinematic_target_points_.push_back(Rigidbody2d());
  kinematic_target_points_.push_back(Rigidbody2d());

  kinematic_target_points_[0].position_ = ofVec2f(50, 100);
  kinematic_target_points_[1].position_ = ofVec2f(ofGetWidth() - 50, 100);
  kinematic_target_points_[2].position_ = ofVec2f(ofGetWidth() - 50, ofGetHeight() - 50);
  kinematic_target_points_[3].position_ = ofVec2f(50, ofGetHeight() - 50);

  current_kinematic_target_index_ = 0;
}

void Hw1App::SetupSeekDemo() { 
  seek_target_point_.MoveOffScreen();
  flocking_centroid_.MoveOffScreen();
  agents_.clear(); 
  agents_.push_back(AiAgent());
  agents_.back().color_ = ofColor(255, 0, 0);
  agents_.back().radius_ = 20;
  agents_.back().rigidbody_.position_.x = 50;
  agents_.back().rigidbody_.position_.y = ofGetHeight() - 50;
  agents_.back().number_of_breadcrumbs_ = 100;
  agents_.back().time_per_breadcrumb_ = 0.25f;
}

void Hw1App::SetupSeekArriveDemo() { 
  seek_target_point_.MoveOffScreen();
  flocking_centroid_.MoveOffScreen();
  agents_.clear();
  agents_.push_back(AiAgent());
  agents_.back().color_ = ofColor(255, 0, 0);
  agents_.back().radius_ = 20;
  agents_.back().rigidbody_.position_.x = 50;
  agents_.back().rigidbody_.position_.y = ofGetHeight() - 50;
  agents_.back().number_of_breadcrumbs_ = 100;
  agents_.back().time_per_breadcrumb_ = 0.25f;
}

void Hw1App::SetupWanderDemo() {
  seek_target_point_.MoveOffScreen();
  flocking_centroid_.MoveOffScreen();
  agents_.clear();
  agents_.push_back(AiAgent());
  agents_.back().color_ = ofColor(255, 0, 0);
  agents_.back().radius_ = 10;
  agents_.back().rigidbody_.position_.x = ofGetWidth() / 2;
  agents_.back().rigidbody_.position_.y = ofGetHeight()/2;
  agents_.back().top_speed_ = 20;
  agents_.back().number_of_breadcrumbs_ = 100;
}

void Hw1App::SetupWander2Demo() {
  seek_target_point_.MoveOffScreen();
  flocking_centroid_.MoveOffScreen();
  agents_.clear();
  agents_.push_back(AiAgent());
  agents_.back().color_ = ofColor(255, 0, 0);
  agents_.back().radius_ = 10;
  agents_.back().rigidbody_.position_.x = ofGetWidth() / 2;
  agents_.back().rigidbody_.position_.y = ofGetHeight() / 2;
  agents_.back().top_speed_ = 20;
  agents_.back().number_of_breadcrumbs_ = 100;
}

void Hw1App::SetupFlockingDemo() {
  seek_target_point_.MoveOffScreen();
  flocking_centroid_.MoveOffScreen();
  agents_.clear();
  agents_.push_back(AiAgent());
  agents_.back().color_ = ofColor(255, 0, 0);
  agents_.back().radius_ = 10;
  agents_.back().rigidbody_.position_.x = ofGetWidth() / 2;
  agents_.back().rigidbody_.position_.y = ofGetHeight() / 2;
  agents_.back().top_speed_ = 75;
  agents_.back().number_of_breadcrumbs_ = 100;
  agents_.back().flocking_mass_ = number_of_flock_followers_;

  float flock_initial_x_delta = 40;
  float flock_initial_y_delta = 40;
  for (unsigned int i = 0; i < number_of_flock_followers_; i++) {
    agents_.push_back(AiAgent());
    agents_.back().color_ = ofColor(0, 255, 0);
    agents_.back().radius_ = 5;
    agents_.back().rigidbody_.position_.x = ofGetWidth() / 2 + (number_of_flock_followers_/2 - i) * flock_initial_x_delta;
    agents_.back().rigidbody_.position_.y = ofGetHeight() / 2 + (number_of_flock_followers_/2 - i) * flock_initial_y_delta;
    agents_.back().top_speed_ = 400;
    agents_.back().number_of_breadcrumbs_ = 5;
    agents_.back().flocking_mass_ = 1;
  }

  flocking_centroid_.arrive_target_radius_ = 5;
  flocking_centroid_.arrive_slow_radius_ = 20;
}

void Hw1App::UpdateKinematicDemo(float dt) {
  if (kinematic_target_points_.empty() || agents_.empty()) {
    // Error
    return;
  }
  float position_delta = 1;
  if (agents_[0].rigidbody_.position_.distance(
          kinematic_target_points_[current_kinematic_target_index_]
              .position_) <= position_delta) {
    current_kinematic_target_index_++;
    current_kinematic_target_index_ %= kinematic_target_points_.size();
  }
  float max_speed = 100;
  agents_[0].UpdateKinematic(
      dt, AiBehaviors::KinematicSeek(
              agents_[0].rigidbody_,
              kinematic_target_points_[current_kinematic_target_index_],
              max_speed));
  // Point in the direction of motion
  agents_[0].rigidbody_.orientation_ = atan2f(
      agents_[0].rigidbody_.velocity_.y, agents_[0].rigidbody_.velocity_.x);
}

void Hw1App::UpdateSeekDemo(float dt) {
  if (agents_.empty() || (seek_target_point_.rigidbody_.position_.x < 0 &&
                          seek_target_point_.rigidbody_.position_.y < 0)) {
    // There is no boid or there is no target
    return;
  }
  float max_linear_accel = 100;
  // Calculate the linear acceleration for the boid
  DynamicSteeringOutput steering_output = AiBehaviors::DynamicSeek(
      agents_[0].rigidbody_, seek_target_point_.rigidbody_, max_linear_accel);
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  steering_output = steering_output + LookInDirectionOfTravel(agents_[0]);
  agents_[0].Update(dt, steering_output);
}

void Hw1App::UpdateSeekArriveDemo(float dt) {
  if (agents_.empty() || (seek_target_point_.rigidbody_.position_.x < 0 &&
                          seek_target_point_.rigidbody_.position_.y < 0)) {
    // There is no boid or there is no target
    return;
  }
  float max_linear_accel = 100;
  float distance_to_switch_to_arrive = seek_target_point_.arrive_slow_radius_ + 10;
  float arrive_time_to_target = 10;

  DynamicSteeringOutput steering_output = DynamicSteeringOutput();
  if (agents_[0].rigidbody_.position_.distance(
          seek_target_point_.rigidbody_.position_) >
      distance_to_switch_to_arrive) {
    // If we are far enough away, use seek to get closer
    steering_output = AiBehaviors::DynamicSeek(
        agents_[0].rigidbody_, seek_target_point_.rigidbody_, max_linear_accel);
  } else {
    // If we are close to the target, use arrive instead
    steering_output = AiBehaviors::DynamicArrive(
        agents_[0].rigidbody_, seek_target_point_.rigidbody_,
        seek_target_point_.arrive_slow_radius_,
        seek_target_point_.arrive_target_radius_, 
        max_linear_accel,
        agents_[0].top_speed_,
        arrive_time_to_target);
  }
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  steering_output = steering_output + LookInDirectionOfTravel(agents_[0]);
  agents_[0].Update(dt, steering_output);
}

void Hw1App::UpdateWanderDemo(float dt) {
  if (agents_.empty()) {
    return;
  }

  float max_rate = PI;
  float offset = 20;
  float max_linear_accel = 100;

  DynamicSteeringOutput steering_output = AiBehaviors::DynamicWander(
      agents_[0].rigidbody_, max_rate, offset, max_linear_accel, wander_target_point_);
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  steering_output = steering_output + LookInDirectionOfTravel(agents_[0]);
  agents_[0].Update(dt, steering_output);
}

void Hw1App::UpdateWander2Demo(float dt) {
  if (agents_.empty()) {
    return;
  }

  float max_rate = PI / 4;
  float offset = 20;
  float max_linear_accel = 100;
  DynamicSteeringOutput steering_output = AiBehaviors::DynamicWander2(
      agents_[0].rigidbody_, max_rate, offset, max_linear_accel, 0.99f, last_wander_target_angle_, wander_target_point_);
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  steering_output = steering_output + LookInDirectionOfTravel(agents_[0]);
  agents_[0].Update(dt, steering_output);
}

void Hw1App::UpdateFlockingDemo(float dt) {
  if (agents_.size() < 2) {
    return;
  }

  float max_rate = PI / 4;
  float offset = 20;
  float max_linear_accel = 100;
  DynamicSteeringOutput steering_output_for_leader =
      AiBehaviors::DynamicWander2(
          agents_[0].rigidbody_, max_rate, offset, max_linear_accel, 0.99f,
          last_wander_target_angle_, wander_target_point_);
  // Calculate the rotational acceleration to face direction of travel and add
  // it to steering_output
  steering_output_for_leader =
      steering_output_for_leader + LookInDirectionOfTravel(agents_[0]);
  agents_[0].Update(dt, steering_output_for_leader);
  // Now update followers
  float personal_radius = 100;
  float decay = 10000;
  float max_time_to_stall = 1;
  float arrive_time_to_target = 1;
  float velocity_match_time_to_target = 1;
  float blending_weight_separate = 1;
  float blending_weight_arrive = 0.8f;
  float blending_weight_velocity_match = 0.6f;
  for (unsigned int i = 1; i < agents_.size(); i++) {
    DynamicSteeringOutput steering_output = AiBehaviors::Flocking(
        agents_[i], agents_, max_linear_accel, personal_radius, decay, dt,
        max_time_to_stall, flocking_centroid_.arrive_slow_radius_,
        flocking_centroid_.arrive_target_radius_, agents_[i].top_speed_,
        arrive_time_to_target, velocity_match_time_to_target,
        blending_weight_separate, blending_weight_velocity_match,
        blending_weight_arrive, flocking_centroid_);
    steering_output = steering_output + LookInDirectionOfTravel(agents_[i]);
    agents_[i].Update(dt, steering_output);
  }
}

} // namespace brooks_hw1
