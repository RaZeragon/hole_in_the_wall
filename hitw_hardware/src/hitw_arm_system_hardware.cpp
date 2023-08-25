#include "hitw_hardware/hitw_arm_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_hitw_arm
{
    hardware_interface::return_type HITWBotSystemPositionOnlyHardware::configure(const hardware_interface::HardwareInfo & info)
    {
        info_ = info

        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        hw_states_.resize(info_.joints.size(), std::numeric_limit<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limit<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"),
                    "Joint '%s' has %d command interfaces found. 1 expected.",
                    joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"),
                    "Joint '%s' has %d command interfaces found. 1 expected.",
                    joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);

                return hardware_interface::return_type::ERROR;
            }
        }

        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface>
    HITWBotSystemPositionOnlyHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
        }

        return command_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    HITWBotSystemPositionOnlyHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }
}