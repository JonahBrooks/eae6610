#include "ai_behaviors.h"

#include "ofVec2f.h"
#include "ofMath.h"

namespace brooks_hw3 {
  KinematicSteeringOutput AiBehaviors::KinematicSeek(Rigidbody2d character,
    Rigidbody2d target,
    float max_speed) {
    KinematicSteeringOutput output = KinematicSteeringOutput();
    ofVec2f seek_vector = target.position_ - character.position_;
    seek_vector.normalize();
    seek_vector *= max_speed;

    output.linear_velocity = seek_vector;
    output.rotational_velocity = 0.0f;

    return output;
  }

  DynamicSteeringOutput AiBehaviors::DynamicSeek(Rigidbody2d character,
    Rigidbody2d target,
    float max_linear_accel) {
    DynamicSteeringOutput output = DynamicSteeringOutput();
    ofVec2f seek_vector = target.position_ - character.position_;
    seek_vector.normalize();
    seek_vector *= max_linear_accel;

    output.linear_acceleration = seek_vector;
    output.rotational_acceleration = 0.0f;

    return output;
  }

  DynamicSteeringOutput AiBehaviors::DynamicArrive(Rigidbody2d character, Rigidbody2d target,
    float slow_radius, float target_radius,
    float max_linear_accel, float max_speed,
    float time_to_target) {

    DynamicSteeringOutput output = DynamicSteeringOutput();

    float distance = character.position_.distance(target.position_);
    float target_speed = 0;

    if (distance > slow_radius) {
      target_speed = max_speed;
    }
    else if (distance > target_radius) {
      target_speed = (distance - target_radius) / (slow_radius - target_radius);
      target_speed *= max_speed;
    }
    else {
      target_speed = 0;
    }

    ofVec2f target_velocity = (target.position_ - character.position_).getNormalized() * target_speed;
    output.linear_acceleration = (target_velocity - character.velocity_) / time_to_target;
    if (output.linear_acceleration.length() > max_linear_accel) {
      output.linear_acceleration.normalize();
      output.linear_acceleration *= max_linear_accel;
    }

    return output;
  }

  DynamicSteeringOutput AiBehaviors::DynamicAlign(
    Rigidbody2d character, Rigidbody2d target, float max_rotation, float slow_angle,
    float target_angle, float time_to_target) {
    DynamicSteeringOutput output = DynamicSteeringOutput();

    float theta_c = character.orientation_;
    float theta_t = target.orientation_;

    float theta_d = theta_t - theta_c;
    while (theta_d >= PI) {
      theta_d -= 2 * PI;
    }
    while (theta_d < -PI) {
      theta_d += 2 * PI;
    }

    float theta_size = abs(theta_d);
    float new_max_rotation = max_rotation;
    if (theta_size > slow_angle) {
      // Leave new_max_rotation as maxRotation
    }
    else if (theta_size > target_angle) {
      new_max_rotation = max_rotation * (theta_size - target_angle) /
        (slow_angle - target_angle);
    }
    else {
      new_max_rotation = 0;
    }

    float rotation_difference = abs(character.rotation_ - new_max_rotation);
    rotation_difference /= time_to_target;
    rotation_difference *= (theta_size / theta_d); // The sign of theta_d
    
    output.rotational_acceleration = rotation_difference;
    return output;
}

DynamicSteeringOutput AiBehaviors::LookWhereYouAreGoing(Rigidbody2d character,
  Rigidbody2d target,
  float max_rotation, float slow_angle,
  float target_angle, float time_to_target) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  if (character.velocity_.length() < 0.001) {
    return output; // Return a 0 rotational_acceleration
  }

  target.orientation_ = atan2f(character.velocity_.y, character.velocity_.x);
  output = DynamicAlign(character, target, max_rotation, slow_angle,
                        target_angle, time_to_target);
  
  return output;
}

DynamicSteeringOutput AiBehaviors::DynamicWander(Rigidbody2d character,
  float max_rate, float offset,
  float max_linear_accel) {
  AiAgent wander_target = AiAgent();
  return DynamicWander(character, max_rate, offset, max_linear_accel,
                        wander_target);
}

DynamicSteeringOutput AiBehaviors::DynamicWander(Rigidbody2d character,
  float max_rate, float offset, 
  float max_linear_accel, AiAgent& out_wander_output) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  float theta_c = character.orientation_;
  float theta_size = abs(max_rate);
  theta_size *=
      (ofRandomuf() - ofRandomuf());  // Get a random binomial between -1 and 1
  ofVec2f target_heading = ofVec2f(offset, 0);
  target_heading.rotateRad(theta_c + theta_size);

  ofVec2f target_pos = character.position_ + target_heading;

  Rigidbody2d target = Rigidbody2d();
  target.position_ = target_pos;

  out_wander_output.rigidbody_ = target;

  output = DynamicSeek(character, target, max_linear_accel);

  return output;
}

DynamicSteeringOutput AiBehaviors::DynamicWander2(Rigidbody2d character,
  float max_rate, float offset, float max_linear_accel, float last_direction_bias,
  float& in_out_last_target_angle) {
  AiAgent wander_target = AiAgent();
  return DynamicWander2(character, max_rate, offset, max_linear_accel, last_direction_bias,
                        in_out_last_target_angle, wander_target);
}

DynamicSteeringOutput AiBehaviors::DynamicWander2(Rigidbody2d character,
  float max_rate, float offset, 
  float max_linear_accel, float last_direction_bias,
  float& in_out_last_target_angle, 
  AiAgent& out_wander_output) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  float theta_c = character.orientation_;
  float positive_bias = last_direction_bias;
  float angle_difference = fmodf(in_out_last_target_angle,2*PI);
  
  // Flip to a negative bias if the last angle was negative
  if (angle_difference < 0 ) {
    positive_bias = 1 - last_direction_bias;
  }
  float theta_size = abs(max_rate);
  // Get a random binomial between -1 and 1, with distribution skewed toward the last direction it landed on
  theta_size *=
      (positive_bias * ofRandomuf() - (1 - positive_bias) * ofRandomuf());  
  ofVec2f target_heading = ofVec2f(offset, 0);
  target_heading.rotateRad(theta_c + theta_size);

  ofVec2f target_pos = character.position_ + target_heading;

  Rigidbody2d target = Rigidbody2d();
  target.position_ = target_pos;

  in_out_last_target_angle = theta_size;
  out_wander_output.rigidbody_ = target;

  output = DynamicSeek(character, target, max_linear_accel);

  return output;
}

DynamicSteeringOutput AiBehaviors::VelocityMatch(Rigidbody2d character,
  Rigidbody2d target,
  float max_linear_accel,
  float time_to_target) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  output.linear_acceleration = target.velocity_ - character.velocity_;
  output.linear_acceleration /= time_to_target;

  if (output.linear_acceleration.length() > max_linear_accel) {
    output.linear_acceleration.normalize();
    output.linear_acceleration *= max_linear_accel;
  }

  return output;
}

DynamicSteeringOutput AiBehaviors::DynamicFlee(Rigidbody2d character, Rigidbody2d target,
  float max_linear_accel) {
  DynamicSteeringOutput output = DynamicSteeringOutput();
  ofVec2f flee_vector = character.position_ - target.position_;
  flee_vector.normalize();
  flee_vector *= max_linear_accel;

  output.linear_acceleration = flee_vector;
  output.rotational_acceleration = 0.0f;

  return output;
}

DynamicSteeringOutput AiBehaviors::DynamicEvade(Rigidbody2d character, Rigidbody2d target,
  float max_linear_accel,
  float personal_radius,
  float decay) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  float strength = 0;
  float distance = character.position_.distance(target.position_);
  if (distance < personal_radius) {
    strength = decay / (distance * distance);
    if (strength > max_linear_accel) {
      strength = max_linear_accel;
    }
  }
  output = DynamicFlee(character, target, strength);

  return output;
}

DynamicSteeringOutput AiBehaviors::DynamicSeparate(Rigidbody2d character,
  std::vector<Rigidbody2d> targets,
  float max_linear_accel,
  float personal_radius, float decay,
  float dt, float max_time_to_stall, float& in_out_watchdog_timer) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  for (unsigned int i = 0; i < targets.size(); i++) {
    output = output + DynamicEvade(character, targets[i], max_linear_accel,
                                   personal_radius, decay);
  }

  if (output.linear_acceleration.length() > max_linear_accel) {
    output.linear_acceleration.normalize();
    output.linear_acceleration *= max_linear_accel;
  } 
  if (output.linear_acceleration.length() < 0.01) {
    in_out_watchdog_timer += dt;
    if (in_out_watchdog_timer > max_time_to_stall) {
      output.linear_acceleration = ofVec2f(max_linear_accel, 0); // Always nudge to the right to get unstuck
      in_out_watchdog_timer = 0;
    }
  } else {
    in_out_watchdog_timer = 0;   
  }

  return output;
}

DynamicSteeringOutput AiBehaviors::Flocking(
    AiAgent character, std::vector<AiAgent> flock_members,
    float max_linear_accel, float separate_personal_radius,
    float separate_decay, float separate_dt,
    float separate_max_time_to_stall, float arrive_slow_radius,
    float arrive_target_radius, float arrive_max_speed,
    float arrive_time_to_target, float velocity_match_time_to_target,
    float blending_weight_separate, float blending_weight_velocity_match,
    float blending_weight_arrive, AiAgent& out_centroid) {
  DynamicSteeringOutput output = DynamicSteeringOutput();

  out_centroid.rigidbody_ = Rigidbody2d();
  std::vector<Rigidbody2d> other_rigidbodies;
  float total_mass = 0;
  for (unsigned int i = 0; i < flock_members.size(); i++) {
    total_mass += flock_members[i].flocking_mass_;
    out_centroid.rigidbody_.position_.x +=
        flock_members[i].flocking_mass_ * flock_members[i].rigidbody_.position_.x;
    out_centroid.rigidbody_.position_.y +=
        flock_members[i].flocking_mass_ * flock_members[i].rigidbody_.position_.y;
    out_centroid.rigidbody_.velocity_.x +=
        flock_members[i].flocking_mass_ * flock_members[i].rigidbody_.velocity_.x;
    out_centroid.rigidbody_.velocity_.y +=
        flock_members[i].flocking_mass_ * flock_members[i].rigidbody_.velocity_.y;
    if (flock_members[i] != character) {
      other_rigidbodies.push_back(flock_members[i].rigidbody_);
    }
  }
  out_centroid.rigidbody_.position_.x /= total_mass;
  out_centroid.rigidbody_.position_.y /= total_mass;
  out_centroid.rigidbody_.velocity_.x /= total_mass;
  out_centroid.rigidbody_.velocity_.y /= total_mass;

  DynamicSteeringOutput separate_output =
      DynamicSeparate(character.rigidbody_, other_rigidbodies, max_linear_accel,
      separate_personal_radius, separate_decay, separate_dt,
      separate_max_time_to_stall,
                      character.separation_watchdog_timer_);
  DynamicSteeringOutput arrive_output =
      DynamicArrive(character.rigidbody_, out_centroid.rigidbody_,
                    arrive_slow_radius, arrive_target_radius, max_linear_accel,
                    arrive_max_speed, arrive_time_to_target);
  DynamicSteeringOutput velocity_match_output =
      VelocityMatch(character.rigidbody_, out_centroid.rigidbody_,
                    max_linear_accel, velocity_match_time_to_target);

  output = separate_output * blending_weight_separate +
           arrive_output * blending_weight_arrive +
           velocity_match_output * blending_weight_velocity_match;
  if (output.linear_acceleration.length() > max_linear_accel) {
    output.linear_acceleration.normalize();
    output.linear_acceleration *= max_linear_accel;
  }
  return output;
}

}