#ifndef BROOKS_HW3_SRC_AI_BEHAVIORS_H
#define BROOKS_HW3_SRC_AI_BEHAVIORS_H

#include <vector>

#include "rigidbody_2d.h"
#include "ai_agent.h"
#include "kinematic_steering_output.h"
#include "dynamic_steering_output.h"

namespace brooks_hw3 {
class AiBehaviors {
 public:
  static KinematicSteeringOutput KinematicSeek(Rigidbody2d character, Rigidbody2d target, float max_speed);
  static DynamicSteeringOutput DynamicSeek(Rigidbody2d character, Rigidbody2d target, float max_linear_accel);
  static DynamicSteeringOutput DynamicArrive(
      Rigidbody2d character, Rigidbody2d target, float slow_radius,
      float target_radius, float max_linear_accel, float max_speed,
      float time_to_target);
  static DynamicSteeringOutput DynamicAlign(
      Rigidbody2d character, Rigidbody2d target, float max_rotation,
      float slow_angle, float target_angle, float time_to_target);
  static DynamicSteeringOutput LookWhereYouAreGoing(
      Rigidbody2d character, Rigidbody2d target, float max_rotation,
      float slow_angle, float target_angle, float time_to_target);

  static DynamicSteeringOutput DynamicWander(Rigidbody2d character,
                                             float max_rate, float offset,
                                             float max_linear_accel);
  static DynamicSteeringOutput DynamicWander(Rigidbody2d character,
                                             float max_rate, float offset,
                                             float max_linear_accel,
                                             AiAgent& out_wander_target);
  static DynamicSteeringOutput DynamicWander2(Rigidbody2d character,
                                              float max_rate, float offset,
                                              float max_linear_accel, float last_direction_bias,
                                              float& in_out_last_target_angle);
  static DynamicSteeringOutput DynamicWander2(Rigidbody2d character,
                                              float max_rate, float offset,
                                              float max_linear_accel, float last_direction_bias,
                                              float& in_out_last_target_angle,
                                              AiAgent& out_wander_target);
  static DynamicSteeringOutput VelocityMatch(Rigidbody2d character,
                                             Rigidbody2d target,
                                             float max_linear_accel,
                                             float time_to_target);
  static DynamicSteeringOutput DynamicFlee(Rigidbody2d character,
                                           Rigidbody2d target,
                                           float max_linear_accel);
  static DynamicSteeringOutput DynamicEvade(Rigidbody2d character,
                                            Rigidbody2d target,
                                            float max_linear_accel, 
                                            float personal_radius,
                                            float decay);
  static DynamicSteeringOutput DynamicSeparate(Rigidbody2d character,
                                               std::vector<Rigidbody2d> targets,
                                               float max_linear_accel,
                                               float personal_radius,
                                               float decay, float dt,
                                               float max_time_to_stall,
                                               float& in_out_watchdog_timer);
  static DynamicSteeringOutput Flocking(
      AiAgent character, std::vector<AiAgent> flock_members,
      float max_linear_accel, float separate_personal_radius,
      float separate_decay, float separate_dt,
      float separate_max_time_to_stall, float arrive_slow_radius,
      float arrive_target_radius, float arrive_max_speed,
      float arrive_time_to_target,
      float velocity_match_time_to_target, float blending_weight_separate,
      float blending_weight_velocity_match, float blending_weight_arrive,
      AiAgent& out_centroid);

 private:
  AiBehaviors() = delete;
  ~AiBehaviors() = delete;
};

}

#endif // BROOKS_HW3_SRC_AI_BEHAVIORS_H