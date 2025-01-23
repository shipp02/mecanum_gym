#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "hardware_interface/system_interface.hpp"

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "termios.h"
#include "serial_driver/serial_driver.hpp"

namespace ias
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct MotorInfo
{
  int addr;
  int cmd_base;
};

// Give speed from -1 to 1
// TODO: Do we need to add some Kinematics to go from rad/s to pwm?
// TODO: Pick up motorinfo from the ros2_control xml tags
std::vector<uint8_t> build_message(MotorInfo m, float speed);
int name_to_idx(const std::string & name);

const int CMD_FORWARD = 0;
const int CMD_BACKWARD = 1;

class ias_mecanum : public hardware_interface::SystemInterface
{

public:
  explicit ias_mecanum();

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // {front_left, front_right, back_right, back_left}
  std::vector<double> vel_commands;
  std::vector<double> prev_vel_commands;
  drivers::common::IoContext ctx;
  drivers::serial_driver::SerialDriver driver;
  std::vector<MotorInfo> motors {
    {128, 0},
    {128, 4},
    {129, 0},
    {129, 4}
  };
};

}
