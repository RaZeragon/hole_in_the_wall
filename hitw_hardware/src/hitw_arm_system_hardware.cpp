#include "hitw_hardware/hitw_arm_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hitw_hardware
{
    hardware_interface::return_type HITWBotSystemPositionOnlyHardware::configure(const hardware_interface::HardwareInfo & info)
    {
        info_ = info;

        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);
        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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

        return state_interfaces;
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

    hardware_interface::return_type HITWBotSystemPositionOnlyHardware::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"), "System starting.");

        for (uint i = 0; i < hw_states_.size(); i++)
        {
            if (std::isnan(hw_states_[i]))
            {
                hw_states_[i] = 0;
                hw_commands_[i] = 0;
            }
            else
            {
                hw_commands_[i] = hw_states_[i];
            }
        }

        status_ = hardware_interface::status::STARTED;

        RCLCPP_INFO(rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"), "System successfully started.");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HITWBotSystemPositionOnlyHardware::stop()
    {
        status_ = hardware_interface::status::STOPPED;
        
        RCLCPP_INFO(rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"), "System successfully stopped.");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HITWBotSystemPositionOnlyHardware::read()
    {
        // RCLCPP_INFO(rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"), "Reading...");

        for (uint i = 0; i < hw_states_.size(); i++)
        {
            // Simulate RRBot's movement
            hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
            // RCLCPP_INFO(rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"), 
            //     "Got state %.5f for joint %d!",
            //     hw_states_[i], 
            //     i);
        }
        // RCLCPP_INFO(rclcpp::get_logger("HITWBotSystemPositionOnlyHardware"), "Joints successfully read!");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HITWBotSystemPositionOnlyHardware::write()
    {
        // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

        for (uint i = 0; i < hw_commands_.size(); i++)
        {
            // Simulate sending commands to the hardware
            // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), 
            //     "Got command %.5f for joint %d!",
            //     hw_commands_[i], 
            //     i);
        }
        // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hitw_hardware::HITWBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)