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

#include "controllers/osc_impedance.h"

#include <memory>

namespace controller {
OSCImpedanceController::OSCImpedanceController() = default;
OSCImpedanceController::~OSCImpedanceController() = default;

OSCImpedanceController::OSCImpedanceController(franka::Model &model) {
  model_ = &model;
}

bool OSCImpedanceController::ParseMessage(const FrankaControlMessage &msg) {

  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }
  std::vector<double> kp_position_array, kp_rotation_array;
  kp_position_array.reserve(3);
  kp_rotation_array.reserve(3);
  for (double kp_i : control_msg_.translational_stiffness()) {
    kp_position_array.push_back(kp_i);
  }
  for (double kp_i : control_msg_.rotational_stiffness()) {
    kp_rotation_array.push_back(kp_i);
  }
  Kp_p.diagonal() << Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
      kp_position_array.data());
  Kp_r.diagonal() << Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
      kp_rotation_array.data());

  std::vector<double> kd_position_array, kd_rotation_array;
  kd_position_array.reserve(3);
  kd_rotation_array.reserve(3);
  for (double kd_i : control_msg_.translational_damping()) {
    kd_position_array.push_back(kd_i);
  }
  for (double kd_i : control_msg_.rotational_damping()) {
    kd_rotation_array.push_back(kd_i);
  }

  if (kd_position_array[0] > 1e-5) {
    Kd_p.diagonal() << Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
        kd_position_array.data());
  } else {
    Kd_p << Kp_p.cwiseSqrt() * 2.0;
  }
  if (kd_rotation_array[0] > 1e-5) {
    Kd_r.diagonal() << Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
        kd_rotation_array.data());
  } else {
    Kd_r << Kp_r.cwiseSqrt() * 2.0;
  }

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  ParseMessageArray<double, 7>(control_msg_.config().residual_mass_vec(), residual_mass_vec_);
  ParseMessageArray<double, 3>(control_msg_.config().residual_tau_translation_vec(), residual_tau_translation_vec_);
  ParseMessageArray<double, 3>(control_msg_.config().residual_tau_rotation_vec(), residual_tau_rotation_vec_);
  ParseMessageArray<double, 7>(control_msg_.config().joint_limits_avoidance(), avoidance_weights_);
  ParseMessageArray<double, 7>(control_msg_.config().nullspace_static_q(), static_q_task_);
  ParseMessageArray<double, 7>(control_msg_.config().joint_tau_limits(), joint_tau_limits_);

  nullspace_stiffness_ = control_msg_.config().nullspace_stiffness();
  coriolis_stiffness_ = control_msg_.config().coriolis_stiffness();

  // std::cout << "OSC joint tau limits: " << joint_tau_limits_.transpose() << std::endl;
  // std::cout << "OSC nullspace stiffness: " << nullspace_stiffness_ << std::endl;

  this->state_estimator_ptr_->ParseMessage(msg.state_estimator_msg());

  return true;
}

void OSCImpedanceController::ComputeGoal(
    const std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info) {
  if (control_msg_.goal().is_delta()) {
    goal_state_info->pos_EE_in_base_frame =
        current_state_info->pos_EE_in_base_frame +
        Eigen::Vector3d(control_msg_.goal().x(), control_msg_.goal().y(),
                        control_msg_.goal().z());
    Eigen::AngleAxisd relative_axis_angle;
    Eigen::Vector3d relative_ori(control_msg_.goal().ax(),
                                 control_msg_.goal().ay(),
                                 control_msg_.goal().az());
    AxisAngle(relative_ori, relative_axis_angle);
    goal_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(relative_axis_angle.toRotationMatrix() *
                           current_state_info->quat_EE_in_base_frame);
  } else {
    goal_state_info->pos_EE_in_base_frame = Eigen::Vector3d(
        control_msg_.goal().x(),
        control_msg_.goal().y(),
        control_msg_.goal().z()
    );
    Eigen::AngleAxisd absolute_axis_angle;
    Eigen::Vector3d absolute_ori(control_msg_.goal().ax(),
                                 control_msg_.goal().ay(),
                                 control_msg_.goal().az());
    AxisAngle(absolute_ori, absolute_axis_angle);
    goal_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(absolute_axis_angle);
  }
}

std::array<double, 7> OSCImpedanceController::Step(
    const franka::RobotState &robot_state,
    const Eigen::Vector3d &desired_pos_EE_in_base_frame,
    const Eigen::Quaterniond &desired_quat_EE_in_base_frame) {

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 7, 1> tau_d;

  std::array<double, 49> mass_array = model_->mass(robot_state);
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  M = M + Eigen::Matrix<double, 7, 7>(residual_mass_vec_.asDiagonal());

  // coriolis and gravity
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  std::array<double, 7> gravity_array = model_->gravity(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  std::array<double, 42> jacobian_array =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  Eigen::MatrixXd jacobian_pos(3, 7);
  Eigen::MatrixXd jacobian_ori(3, 7);
  jacobian_pos << jacobian.block(0, 0, 3, 7);
  jacobian_ori << jacobian.block(3, 0, 3, 7);

  // End effector pose in base frame
  Eigen::Affine3d T_EE_in_base_frame(
      Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
  Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());

  // Nullspace goal
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

  // Joint velocity
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // Update the specified state estimator
  if (this->state_estimator_ptr_->IsFirstState()) {
    this->state_estimator_ptr_->Initialize(q, dq, pos_EE_in_base_frame,
                                           quat_EE_in_base_frame);
  } else {
    this->state_estimator_ptr_->Update(q, dq, pos_EE_in_base_frame,
                                       quat_EE_in_base_frame);
  }

  Eigen::Matrix<double, 7, 1> current_q, current_dq;
  // Get state from a specified state estimator
  current_q = this->state_estimator_ptr_->GetCurrentJointPos();
  current_dq = this->state_estimator_ptr_->GetCurrentJointVel();

  // Get eef states from a specified state estimator
  pos_EE_in_base_frame = this->state_estimator_ptr_->GetCurrentEEFPos();
  quat_EE_in_base_frame = this->state_estimator_ptr_->GetCurrentEEFQuat();

  if (desired_quat_EE_in_base_frame.coeffs().dot(
          quat_EE_in_base_frame.coeffs()) < 0.0) {
    quat_EE_in_base_frame.coeffs() << -quat_EE_in_base_frame.coeffs();
  }

  Eigen::Vector3d pos_error;

  pos_error << desired_pos_EE_in_base_frame - pos_EE_in_base_frame;
  Eigen::Quaterniond quat_error(desired_quat_EE_in_base_frame.inverse() *
                                quat_EE_in_base_frame);
  Eigen::Vector3d ori_error;
  ori_error << quat_error.x(), quat_error.y(), quat_error.z();
  ori_error << -T_EE_in_base_frame.linear() * ori_error;

  // Compute matrices
  Eigen::Matrix<double, 7, 7> M_inv(M.inverse());
  Eigen::MatrixXd Lambda_inv(6, 6);
  Lambda_inv << jacobian * M_inv * jacobian.transpose();
  Eigen::MatrixXd Lambda(6, 6);
  control_utils::PInverse(Lambda_inv, Lambda);

  Eigen::Matrix<double, 7, 6> J_inv;
  J_inv << M_inv * jacobian.transpose() * Lambda;
  Eigen::Matrix<double, 7, 7> Nullspace;
  Nullspace << Eigen::MatrixXd::Identity(7, 7) -
                   jacobian.transpose() * J_inv.transpose();

  // Decoupled mass matrices
  Eigen::MatrixXd Lambda_pos_inv(3, 3);
  Lambda_pos_inv << jacobian_pos * M_inv * jacobian_pos.transpose();
  Eigen::MatrixXd Lambda_ori_inv(3, 3);
  Lambda_ori_inv << jacobian_ori * M_inv * jacobian_ori.transpose();

  Eigen::MatrixXd Lambda_pos(3, 3);
  Eigen::MatrixXd Lambda_ori(3, 3);
  control_utils::PInverse(Lambda_pos_inv, Lambda_pos);
  control_utils::PInverse(Lambda_ori_inv, Lambda_ori);

  pos_error = pos_error.unaryExpr([](double x) { return (abs(x) < 1e-4) ? 0. : x; });
  ori_error = ori_error.unaryExpr([](double x) { return (abs(x) < 5e-3) ? 0. : x; });

  // std::cout << "OSC::pos_error " << pos_error.transpose() << std::endl;
  // std::cout << "OSC::ori_error " << ori_error.transpose() << std::endl;

  tau_d << jacobian_pos.transpose() * (Lambda_pos * (Kp_p * pos_error - Kd_p * (jacobian_pos * current_dq))) +
           jacobian_ori.transpose() * (Lambda_ori * (Kp_r * ori_error - Kd_r * (jacobian_ori * current_dq)));

  // std::cout << "res_tau_T = " << residual_tau_translation_vec_.transpose() << std::endl;
  // std::cout << "res_tau_R = " << residual_tau_rotation_vec_.transpose() << std::endl;
  tau_d << tau_d + jacobian_pos.transpose() * residual_tau_translation_vec_ + jacobian_ori.transpose() * residual_tau_rotation_vec_;

  if (getGlobalHandler()->log_controller) {
    std::cout << "OSC::q: " << q.transpose() << std::endl;
    std::cout << "OSC::q_SE: " << current_q.transpose() << std::endl;
    std::cout << "OSC::dq: " << dq.transpose() << std::endl;
    std::cout << "OSC::dq_SE: " << current_dq.transpose() << std::endl;
    std::cout << "OSC::tau_d: " << tau_d.transpose() << std::endl;
  }

  // nullspace control
  if (nullspace_stiffness_ > 0.0) {
    tau_d << tau_d + Nullspace * (static_q_task_ - current_q) * nullspace_stiffness_;
  }
  // std::cout << "OSC nullspace stiffness: " << nullspace_stiffness_ << std::endl;

  if (coriolis_stiffness_ > 0.0) {
    tau_d << tau_d + Nullspace * coriolis * coriolis_stiffness_;
  }

  // Add joint avoidance potential
  Eigen::Matrix<double, 7, 1> avoidance_force;
  avoidance_force.setZero();
  Eigen::Matrix<double, 7, 1> dist2joint_max;
  Eigen::Matrix<double, 7, 1> dist2joint_min;

  dist2joint_max = joint_max_.matrix() - current_q;
  dist2joint_min = current_q - joint_min_.matrix();

  for (int i = 0; i < 7; i++) {
    if (dist2joint_max[i] < 0.25 && dist2joint_max[i] > 0.1)
      avoidance_force[i] += -avoidance_weights_[i] * dist2joint_max[i];
    if (dist2joint_min[i] < 0.25 && dist2joint_min[i] > 0.1)
      avoidance_force[i] += avoidance_weights_[i] * dist2joint_min[i];
  }
  tau_d << tau_d + Nullspace * avoidance_force;
  for (int i = 0; i < 7; i++) {
    if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.)
      tau_d[i] = 0.;
    if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.)
      tau_d[i] = 0.;
  }

  // std::cout << "OSC avoidance weights: " << avoidance_weights_.transpose() << std::endl;

  if (joint_tau_limits_(0) > 1e-3) {
    bool has_saturation = false;
    for (int i = 0; i < 7; ++i) {
      if (std::abs(tau_d[i]) > joint_tau_limits_[i]) {
        has_saturation = true;
        break;
      }
    }
    if (has_saturation) {
      std::cout << "OSC::Joint torque saturation!" << "tau_d: " << tau_d.transpose() << std::endl;
    }

    LimitAbsoluteValue(tau_d, joint_tau_limits_);
  }

  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  std::chrono::high_resolution_clock::time_point t2 =
      std::chrono::high_resolution_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  // std::cout << "OSC step took: " << time.count() << " ms"  << std::endl;
  return tau_d_array;
}
} // namespace controller
