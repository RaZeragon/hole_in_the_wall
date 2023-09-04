#ifndef ROS2_CONTROL_HITW_ARM_SYSTEM_HARDWARE
#define ROS2_CONTROL_HITW_ARM_SYSTEM_HARDWARE

#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"

namespace hitw_hardware
{
    class HITWBotSystemPositionOnlyHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(HITWBotSystemPositionOnlyHardware)

            hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type start() override;
            hardware_interface::return_type stop() override;
            hardware_interface::return_type read() override;
            hardware_interface::return_type write() override;

        private:
            //double hw_start_sec_;
            //double hw_stop_sec_;
            double hw_slowdown_;

            std::vector<double> hw_commands_;
            std::vector<double> hw_states_;
    };
}

#endif