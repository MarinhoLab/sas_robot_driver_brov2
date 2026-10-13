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
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver_brov2/sas_robot_driver_brov2.hpp>
#include <dqrobotics/utils/DQ_Math.h>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);

    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_brov2");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverBrov2Configuration configuration; // create an instance (object) of the struct

    	// Temporary containers to read ROS params
    	double batt_voltage;

		std::vector<int64_t> thruster_channels;
		std::vector<int64_t> thruster_neutral_pwm;
		std::vector<int64_t> servo_function_values;
		std::vector<double>  thruster_min_pwm;
		std::vector<double>  thruster_max_pwm;
		std::vector<double>  thruster_min_force;
		std::vector<double>  thruster_max_force;
		std::vector<double>  thruster_deadband_pwm;
    	std::vector<double>  fwd_thrust_curve_coeffs;
    	std::vector<double>  rev_thrust_curve_coeffs;
		std::vector<double> x, y, z, rx, ry, rz;
		std::vector<std::string> servo_function_params;

    	// Reads ros2 parameters into c++ variables
		sas::get_ros_parameter(node, "rc_override_topic",  configuration.rc_override_topic);
		sas::get_ros_parameter(node, "param_service_name", configuration.param_service_name);

    	sas::get_ros_parameter(node, "batt_voltage", batt_voltage);
		sas::get_ros_parameter(node, "thruster_channels", thruster_channels);
		sas::get_ros_parameter(node, "thruster_neutral_pwm", thruster_neutral_pwm);
		sas::get_ros_parameter(node, "thruster_min_pwm", thruster_min_pwm);
		sas::get_ros_parameter(node, "thruster_max_pwm", thruster_max_pwm);
		sas::get_ros_parameter(node, "thruster_min_force", thruster_min_force);
		sas::get_ros_parameter(node, "thruster_max_force", thruster_max_force);
		sas::get_ros_parameter(node, "thruster_deadband_pwm", thruster_deadband_pwm);
		sas::get_ros_parameter(node, "servo_function_params", servo_function_params);
		sas::get_ros_parameter(node, "servo_function_values", servo_function_values);
    	sas::get_ros_parameter(node, "fwd_thrust_curve_coeffs", fwd_thrust_curve_coeffs);
    	sas::get_ros_parameter(node, "rev_thrust_curve_coeffs", rev_thrust_curve_coeffs);


    	sas::get_ros_parameter(node, "tam.x",  x);
    	sas::get_ros_parameter(node, "tam.y",  y);
    	sas::get_ros_parameter(node, "tam.z",  z);
    	sas::get_ros_parameter(node, "tam.rx", rx);
    	sas::get_ros_parameter(node, "tam.ry", ry);
    	sas::get_ros_parameter(node, "tam.rz", rz);

		// Validate equal lengths
		const size_t n = thruster_channels.size();
		if (thruster_neutral_pwm.size() != n ||
		thruster_min_pwm.size()     != n ||
		thruster_max_pwm.size()     != n ||
		servo_function_params.size()!= n)
		{
		 throw std::runtime_error("Thruster parameter arrays must have the same length.");
		}

    	const size_t N = x.size();
    	if (y.size()!=N||z.size()!=N||rx.size()!=N||ry.size()!=N||rz.size()!=N)
    		throw std::runtime_error("TAM row lengths must match.");
    	if (N!=n) {
    		throw std::runtime_error("TAM column count must equal number of thrusters.");
		}

		// Read curve coefficients
    	configuration.v_bus = batt_voltage;
    	configuration.fwd_thrust_curve_coeffs = fwd_thrust_curve_coeffs;
    	configuration.rev_thrust_curve_coeffs = rev_thrust_curve_coeffs;
    	if (configuration.fwd_thrust_curve_coeffs.size() != 5 || configuration.rev_thrust_curve_coeffs.size() != 5)
					throw std::runtime_error("thrust_curve_coefficients must have length 5.");

    	// Prepare thruster_configs to be populated
		configuration.thruster_configs.clear();
		configuration.thruster_configs.reserve(n);

    	// Pack TAM rows into an eigen allocation matrix
    	configuration.B.resize(6, static_cast<Eigen::Index>(N));
    	configuration.B.row(0) = Eigen::Map<Eigen::VectorXd>(x.data(),  N).transpose();
    	configuration.B.row(1) = Eigen::Map<Eigen::VectorXd>(y.data(),  N).transpose();
    	configuration.B.row(2) = Eigen::Map<Eigen::VectorXd>(z.data(),  N).transpose();
    	configuration.B.row(3) = Eigen::Map<Eigen::VectorXd>(rx.data(), N).transpose();
    	configuration.B.row(4) = Eigen::Map<Eigen::VectorXd>(ry.data(), N).transpose();
    	configuration.B.row(5) = Eigen::Map<Eigen::VectorXd>(rz.data(), N).transpose();

    	// Convert arrays into per thruster config structs
		for (size_t i = 0; i < thruster_channels.size(); ++i)
		{
		    configuration.thruster_configs.push_back({
			static_cast<int>(thruster_channels[i]),
			static_cast<int>(thruster_neutral_pwm[i]),
			static_cast<int>(servo_function_values[i]),
			thruster_min_force[i],
			thruster_max_force[i],
			thruster_min_pwm[i],
			thruster_max_pwm[i],
			thruster_deadband_pwm[i],
			servo_function_params[i]
		    });
		}

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating UnderwaterRobotDriver.");
        auto robot_driver_myrobot = std::make_shared<sas::RobotDriverBrov2>(configuration,
                                                                    &kill_this_process);  // create an instance (object) of the driver class

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_myrobot,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);  // create an instance(object) of the SAS ROS wrapper class
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    return 0;
}
