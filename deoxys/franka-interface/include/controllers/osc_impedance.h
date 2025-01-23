// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_H_

namespace controller {
class OSCImpedanceController : public BaseController {
protected:
  FrankaOSCPoseControllerMessage control_msg_;
  Eigen::Matrix<double, 3, 3> Kp_p, Kp_r, Kd_p, Kd_r;

  Eigen::Matrix<double, 7, 1> residual_mass_vec_;
  Eigen::Matrix<double, 3, 1> residual_tau_translation_vec_;
  Eigen::Matrix<double, 3, 1> residual_tau_rotation_vec_;

  Eigen::Matrix<double, 7, 1> static_q_task_;
  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;
  Eigen::Array<double, 7, 1> avoidance_weights_;
  Eigen::Array<double, 7, 1> joint_tau_limits_;

  Eigen::Matrix<double, 7, 1> diff_ik_kp_, diff_ik_kd_;

  double coriolis_stiffness_;
  double nullspace_stiffness_;
  bool use_diff_ik_;

public:
  OSCImpedanceController();
  OSCImpedanceController(franka::Model &model);

  ~OSCImpedanceController();

  bool ParseMessage(const FrankaControlMessage &msg);

  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info);

  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Vector3d &,
                             const Eigen::Quaterniond &);
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_H_
