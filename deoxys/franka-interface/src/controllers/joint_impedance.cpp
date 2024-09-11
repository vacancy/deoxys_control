// Copyright 2022 Yifeng Zhu

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"

#include "controllers/joint_impedance.h"

#include <memory>

namespace controller {
JointImpedanceController::JointImpedanceController() = default;
JointImpedanceController::~JointImpedanceController() = default;

JointImpedanceController::JointImpedanceController(franka::Model &model) {
  model_ = &model;
}

bool JointImpedanceController::ParseMessage(const FrankaControlMessage &msg) {

  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  ParseMessageArray<double, 7>(control_msg_.kp(), Kp);
  ParseMessageArray<double, 7>(control_msg_.kd(), Kd);
  ParseMessageArray<double, 7>(control_msg_.joint_tau_limits(), joint_tau_limits_);

  this->state_estimator_ptr_->ParseMessage(msg.state_estimator_msg());
  return true;
}

void JointImpedanceController::ComputeGoal(
    const std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info) {
  if (control_msg_.goal().is_delta()) {
    Eigen::Matrix<double, 7, 1> delta_joint_position;
    delta_joint_position << control_msg_.goal().q1(), control_msg_.goal().q2(),
        control_msg_.goal().q3(), control_msg_.goal().q4(),
        control_msg_.goal().q5(), control_msg_.goal().q6(),
        control_msg_.goal().q7();
    goal_state_info->joint_positions =
        current_state_info->joint_positions + delta_joint_position;
  } else {
    goal_state_info->joint_positions << control_msg_.goal().q1(),
        control_msg_.goal().q2(), control_msg_.goal().q3(),
        control_msg_.goal().q4(), control_msg_.goal().q5(),
        control_msg_.goal().q6(), control_msg_.goal().q7();
  }
  // goal_state_info->joint_positions << control_msg_.goal().q1(),
  // control_msg_.goal().q2(), control_msg_.goal().q3(),
  // control_msg_.goal().q4(), control_msg_.goal().q5(),
  // control_msg_.goal().q6(), control_msg_.goal().q7();
}

std::array<double, 7>
JointImpedanceController::Step(const franka::RobotState &robot_state,
                               const Eigen::Matrix<double, 7, 1> &desired_q) {

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 7, 1> tau_d;

  std::array<double, 49> mass_array = model_->mass(robot_state);
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  // coriolis and gravity
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // Current joint velocity
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // Current joint position
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

  Eigen::MatrixXd joint_pos_error(7, 1);

  Eigen::Matrix<double, 7, 1> current_q, current_dq;

  if (this->state_estimator_ptr_->IsFirstState()) {
    this->state_estimator_ptr_->Initialize(q, dq);
  } else {
    this->state_estimator_ptr_->Update(q, dq);
  }

  // current_q_ and current_dq_ will be raw data if estimation flag is set to
  // false
  current_q = this->state_estimator_ptr_->GetCurrentJointPos();
  current_dq = this->state_estimator_ptr_->GetCurrentJointVel();
  joint_pos_error << desired_q - current_q;

  tau_d << Kp.cwiseProduct(joint_pos_error) - Kd.cwiseProduct(current_dq);
  // joint_pos_error << desired_q_ - q;
  // tau_d << Kp.cwiseProduct(joint_pos_error) - Kd.cwiseProduct(dq);

  if (getGlobalHandler()->log_controller) {
    std::cout << "JI::q: " << q.transpose() << std::endl;
    std::cout << "JI::dq: " << dq.transpose() << std::endl;
    std::cout << "JI::qh: " << desired_q.transpose() << std::endl;
    std::cout << "JI::err: " << joint_pos_error.transpose() << std::endl;
    std::cout << "JI::tau: " << tau_d.transpose() << std::endl;
  }

  Eigen::Matrix<double, 7, 1> dist2joint_max;
  Eigen::Matrix<double, 7, 1> dist2joint_min;

  dist2joint_max = joint_max_.matrix() - current_q;
  dist2joint_min = current_q - joint_min_.matrix();

  for (int i = 0; i < 7; i++) {
    if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.)
      tau_d[i] = 0.;
    if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.)
      tau_d[i] = 0.;
  }

  if (joint_tau_limits_(0) > 1e-3) {
    LimitAbsoluteValue(tau_d, joint_tau_limits_);
  }
  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  return tau_d_array;
}
} // namespace controller
