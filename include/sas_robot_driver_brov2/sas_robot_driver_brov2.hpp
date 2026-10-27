#pragma once
/*
# Copyright (c) 2024-2026 Rawan Mohamed Elsayed and Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_brov2.
#
#    sas_robot_driver_brov2 is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 2.1 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_brov2 is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_brov2.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Rawan M. Elsayed, email: rawan.elsayed@manchester.ac.uk
#   Co-author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <atomic>
#include <memory>
#include <sas_core/sas_robot_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <realtime_tools/realtime_publisher.hpp>


/**
 * @file sas_robot_driver_brov2.hpp
 * @brief SAS robot driver for the BlueROV2 and BlueROV2 Heavy, using MAVROS RC override.
 */

namespace sas
{

/**
 * @brief Configuration of a single thruster.
 */
struct ThrusterConfig
{
    int    channel;                   ///< RC channel index (1-based)
    int    neutral_pwm;               ///< PWM to send when stopped
    int    servo_function_values;     ///< Servo function value (1 is passthrough mode)

    double f_min;                     ///< Min allowed thruster force
    double f_max;                     ///< Max allowed thruster force
    double min_pwm;                   ///< Min allowed PWM
    double max_pwm;                   ///< Max allowed PWM
    double deadband_pwm;              ///< PWM deadband

    std::string servo_function_param; ///< Autopilot parameter name, e.g. "SERVO1_FUNCTION"
};

/**
 * @brief Configuration of the RobotDriverBrov2.
 */
struct RobotDriverBrov2Configuration
{
    std::vector<ThrusterConfig> thruster_configs;                    ///< One ThrusterConfig per thruster
    std::vector<double> fwd_thrust_curve_coeffs;                     ///< Forward thrust curve coefficients
    std::vector<double> rev_thrust_curve_coeffs;                     ///< Reverse thrust curve coefficients

    std::string pwm_mapping;                                         ///< Thrust-to-PWM mapping method (linear or bivariate)
    std::string rc_override_topic{"mavros/rc/override"};             ///< Topic the RC override is published to
    std::string node_name_prefix{"sas_thruster_node"};               ///< Prefix of the internal node name
    std::string param_service_name{"/mavros/param/set_parameters"};  ///< MAVROS parameter service
    std::string sim_batt_voltage_param{"SIM_BATT_VOLTAGE"};          ///< Simulated battery voltage parameter

    double v_bus;                                                    ///< Bus (battery) voltage

	bool set_battery_voltage{false};
	bool use_battery_telemetry{false};

    Eigen::MatrixXd B;                                               ///< Thruster allocation matrix (6 x number of thrusters)
};

/**
 * @brief SAS driver for the BlueROV2. The "joint torques" interface is a 6D wrench
 *        that is allocated to thrusters and sent as PWM through MAVROS RC override.
 *        The position interface is not supported.
 */
class RobotDriverBrov2: public RobotDriver
{
private:
    RobotDriverBrov2Configuration configuration_;

    //Use the Impl idiom to "hide" internal driver sources
    class Impl;
    std::unique_ptr<Impl> impl_;
public:

    // Prevent copies as usually drivers have threads
    RobotDriverBrov2(const RobotDriverBrov2&)=delete;
    RobotDriverBrov2()=delete;
    ~RobotDriverBrov2();

    /**
     * @brief Constructor.
     * @param configuration The driver configuration.
     * @param break_loops Pointer to a flag that, when true, breaks the driver loops.
     */
    RobotDriverBrov2(const RobotDriverBrov2Configuration &configuration, std::atomic_bool* break_loops);

    /**
     * @brief Get the last commanded PWM vector (there is no sensor feedback).
     * @return The last commanded PWM, one entry per thruster.
     * @throws std::runtime_error if not connected or not yet commanded.
     */
    Eigen::VectorXd get_joint_positions() override;

    /**
     * @brief Not supported.
     * @throws std::runtime_error always. Use set_target_joint_torques().
     */
    void set_target_joint_positions(const Eigen::VectorXd& q) override;

    /**
     * @brief Get the last commanded wrench.
     * @return The last commanded 6D wrench.
     * @throws std::runtime_error if not connected, not initialized, or not yet commanded.
     */
    Eigen::VectorXd get_joint_torques() override;

    /**
     * @brief Command a 6D wrench, which is allocated to the thrusters and sent as PWM.
     * @param tau The desired 6D wrench.
     * @throws std::runtime_error if not connected/initialized or if tau is not 6D.
     */
    void set_target_joint_torques(const Eigen::VectorXd& tau) override;

    /// @brief Create the ROS node and the RC override publisher.
    void connect() override;
    /// @brief Release the ROS resources created in connect().
    void disconnect() override;

    /// @brief Configure the autopilot servo parameters and send neutral commands.
    void initialize() override;
    /// @brief Return the thrusters to neutral.
    void deinitialize() override;

};
}
