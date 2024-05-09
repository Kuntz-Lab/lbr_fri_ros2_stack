#include "lbr_fri_ros2/command_guard.hpp"

namespace lbr_fri_ros2 {
CommandGuard::CommandGuard(const CommandGuardParameters &command_guard_parameters)
    : parameters_(command_guard_parameters){};

bool CommandGuard::is_valid_command(const_idl_command_t_ref lbr_command,
                                    const_fri_state_t_ref lbr_state) {
  switch (lbr_state.getClientCommandMode()) {
  case KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE:
    return false;
#if FRICLIENT_VERSION_MAJOR == 1
  case KUKA::FRI::EClientCommandMode::POSITION:
#endif
#if FRICLIENT_VERSION_MAJOR == 2
  case KUKA::FRI::EClientCommandMode::JOINT_POSITION:
#endif
  case KUKA::FRI::EClientCommandMode::TORQUE:
  case KUKA::FRI::EClientCommandMode::WRENCH:
    if (!command_in_position_limits_(lbr_command, lbr_state)) {
      return false;
    }
    if (!command_in_velocity_limits_(lbr_command, lbr_state)) {
      return false;
    }
    return true;
#if FRICLIENT_VERSION_MAJOR == 2
  case KUKA::FRI::EClientCommandMode::CARTESIAN_POSE:
    return false;
#endif
  default:
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid EClientCommandMode provided");
    return false;
  }
  return false;
}

void CommandGuard::log_info() const {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "*** Parameters:");
  for (std::size_t i = 0; i < parameters_.joint_names.size(); ++i) {
    RCLCPP_INFO(
        rclcpp::get_logger(LOGGER_NAME),
        "*   Joint %s limits: Position: [%.1f, %.1f] deg, velocity: %.1f deg/s, torque: %.1f Nm",
        parameters_.joint_names[i].c_str(), parameters_.min_position[i] * (180. / M_PI),
        parameters_.max_position[i] * (180. / M_PI), parameters_.max_velocity[i] * (180. / M_PI),
        parameters_.max_torque[i]);
  }
}

bool CommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                               const_fri_state_t_ref /*lbr_state*/) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] < parameters_.min_position[i] ||
        lbr_command.joint_position[i] > parameters_.max_position[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Position not in limits for joint '"
                                             << parameters_.joint_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool CommandGuard::command_in_velocity_limits_(const_idl_command_t_ref lbr_command,
                                               const_fri_state_t_ref lbr_state) {
  const double &dt = lbr_state.getSampleTime();
  if (!prev_measured_joint_position_init_) {
    prev_measured_joint_position_init_ = true;
    std::memcpy(prev_measured_joint_position_.data(), lbr_state.getMeasuredJointPosition(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    return true;
  }
  for (std::size_t i = 0; i < lbr_command.joint_position[i]; ++i) {
    if (std::abs(prev_measured_joint_position_[i] - lbr_state.getMeasuredJointPosition()[i]) / dt >
        parameters_.max_velocity[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Velocity not in limits for joint '"
                                             << parameters_.joint_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  std::memcpy(prev_measured_joint_position_.data(), lbr_state.getMeasuredJointPosition(),
              sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  return true;
}

bool CommandGuard::command_in_torque_limits_(const_idl_command_t_ref lbr_command,
                                             const_fri_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.torque.size(); ++i) {
    if (std::abs(lbr_command.torque[i] + lbr_state.getExternalTorque()[i]) >
        parameters_.max_torque[i]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME), ColorScheme::ERROR
                                                               << "Torque not in limits for joint '"
                                                               << parameters_.joint_names[i].c_str()
                                                               << "'" << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

bool SafeStopCommandGuard::command_in_position_limits_(const_idl_command_t_ref lbr_command,
                                                       const_fri_state_t_ref lbr_state) const {
  for (std::size_t i = 0; i < lbr_command.joint_position.size(); ++i) {
    if (lbr_command.joint_position[i] <
            parameters_.min_position[i] + parameters_.max_velocity[i] * lbr_state.getSampleTime() ||
        lbr_command.joint_position[i] >
            parameters_.max_position[i] - parameters_.max_velocity[i] * lbr_state.getSampleTime()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                          ColorScheme::ERROR << "Position not in limits for joint '"
                                             << parameters_.joint_names[i].c_str() << "'"
                                             << ColorScheme::ENDC);
      return false;
    }
  }
  return true;
}

std::unique_ptr<CommandGuard>
command_guard_factory(const CommandGuardParameters &command_guard_parameters,
                      const std::string &variant) {
  constexpr char LOGGER_NAME[] = "lbr_fri_ros2::command_guard_factory";
  if (variant == "default") {
    return std::make_unique<CommandGuard>(command_guard_parameters);
  }
  if (variant == "safe_stop") {
    return std::make_unique<SafeStopCommandGuard>(command_guard_parameters);
  }
  std::string err = "Invalid CommandGuard variant provided.";
  RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME),
                      ColorScheme::ERROR << err.c_str() << ColorScheme::ENDC);
  throw std::runtime_error(err);
}
} // end of namespace lbr_fri_ros2
