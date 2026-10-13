#pragma once
/*
# Copyright (c) 2024-2026 Rawan Mohamed Elsayed and Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_brov2.
#
#    sas_robot_driver_brov2 is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
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

using namespace Eigen;

namespace sas
{

struct ThrusterConfig
{
    int    channel;                   // RC channel index
    int    neutral_pwm;               // PWM to send when stopped
	int    servo_function_values;     // Passthrough mode

	double f_min;					  // Min allowed thruster force
	double f_max;					  // Max allowed thruster force
    double min_pwm;                   // Min allowed PWM
    double max_pwm;                   // Max allowed PWM
	double deadband_pwm;

	std::string servo_function_param; // e.g. "SERVO1_FUNCTION"
};

struct RobotDriverBrov2Configuration
{
	std::vector<ThrusterConfig> thruster_configs;                    // holds ThrusterConfig objects per thruster
	std::vector<double> fwd_thrust_curve_coeffs;
	std::vector<double> rev_thrust_curve_coeffs;

	std::string rc_override_topic{"mavros/rc/override"};             // topic the publisher publishes to
	std::string node_name_prefix{"sas_thruster_node"};               // node
	std::string param_service_name{"/mavros/param/set_parameters"};  // service client we connect to
	std::string sim_batt_voltage_param{"SIM_BATT_VOLTAGE"};

	double v_bus;

	Eigen::MatrixXd B;
};

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

    // This boilderplate constructor usually does the job well and prevent big changes when
    // parameters change
    RobotDriverBrov2(const RobotDriverBrov2Configuration &configuration, std::atomic_bool* break_loops);

    /// Everything below this line is an override
    /// the concrete implementations are needed
	VectorXd get_joint_positions() override;
	void set_target_joint_positions(const VectorXd& q) override;

	VectorXd get_joint_torques() override;
    void set_target_joint_torques(const VectorXd& tau) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

};
}
