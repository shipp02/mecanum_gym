#include "mecanum/ias_mecanum.hpp"
#include <iostream>

namespace ias
{

std::vector<uint8_t> build_message(MotorInfo m, float speed)
{
  uint8_t cmd = m.cmd_base;
  if(speed < 0) {
    cmd += CMD_BACKWARD;
    speed *= -1;
  } else {
    cmd += CMD_FORWARD;
  }
  uint8_t m_speed = speed * 127;

  std::vector<uint8_t> msg{m.addr, cmd, m_speed, 0};
  // Checksum
  msg[3] = (msg[0] + msg[1] + msg[2]) & 0x7f;

  return msg;
}

int name_to_idx(const std::string & name)
{
  if(name == "front_left") {
    return 0;
  } else if (name == "front_right") {
    return 1;
  } else if (name == "back_right") {
    return 2;
  } else if (name == "back_left") {
    return 3;
  } else {
    throw std::runtime_error(
        "name must be one of {front_left, front_right, back_right, back_left}");
  }
}

ias_mecanum::ias_mecanum()
: driver{ctx}
{
  std::cout << "Hardware Interface Init" << std::endl;
}

CallbackReturn ias_mecanum::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  driver.init_port("/dev/ttyTHS0",
     drivers::serial_driver::SerialPortConfig(9600,
     drivers::serial_driver::FlowControl::NONE,
     drivers::serial_driver::Parity::NONE,
     drivers::serial_driver::StopBits::ONE
     )
  );
  return CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> ias_mecanum::export_state_interfaces()
{
  return std::vector<hardware_interface::StateInterface>{};
}
std::vector<hardware_interface::CommandInterface> ias_mecanum::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    int m_idx = name_to_idx(info_.joints[i].name);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &vel_commands[m_idx]
    )
    );
    motors[m_idx].addr = std::atoi(info_.joints[i].parameters.find("addr")->second.c_str());
    int motor_num = std::atoi(info_.joints[i].parameters.find("motor")->second.c_str());
    motors[m_idx].cmd_base = 0 ? motor_num == 1 : 4;
    std::cout << info_.joints[i].name << " addr:" << motors[m_idx].addr
              << "cmd_base:" << motors[m_idx].cmd_base << std::endl;
  }
  return command_interfaces;
}
hardware_interface::return_type ias_mecanum::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type ias_mecanum::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}

CallbackReturn ias_mecanum::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  vel_commands.resize(info_.joints.size());
  prev_vel_commands.resize(info_.joints.size());
  if(!driver.port()->is_open()) {
    return CallbackReturn::ERROR;
  }


  return CallbackReturn::SUCCESS;
}

CallbackReturn ias_mecanum::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  driver.port()->close();
  return CallbackReturn::SUCCESS;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ias::ias_mecanum, hardware_interface::SystemInterface)
