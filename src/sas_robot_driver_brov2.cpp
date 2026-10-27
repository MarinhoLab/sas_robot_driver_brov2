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


#include "sas_robot_driver_brov2/sas_robot_driver_brov2.hpp"
#include "sas_robot_driver_brov2/driver_thruster_mapping.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <sas_core/eigen3_std_conversions.hpp>
#include <mavros_msgs/msg/state.hpp>

namespace sas
{

class RobotDriverBrov2::Impl
{

public:
    bool connected{false};
    bool motor_on{false};
    bool initialized{false};
    bool mavros_connected{false};

	Eigen::MatrixXd B_pinv;
	Eigen::VectorXd last_pwm_;
	Eigen::VectorXd last_tau_;

    std::shared_ptr<rclcpp::Node> node;  // defines node as a Impl member variable and it points to the ROS object -> rclcpp::Node
    std::shared_ptr<rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>> override_rc_pub;  //multiple things can own/point to the object
    std::unique_ptr<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>> rt_override_rc_pub;  // only one owner
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;

    Impl()
        {

        }

};

RobotDriverBrov2::RobotDriverBrov2(const RobotDriverBrov2Configuration& configuration, std::atomic_bool* break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
   // joint_limits_ = configuration.joint_limits; //for the superclass
    impl_ = std::make_unique<RobotDriverBrov2::Impl>();

    const int n = static_cast<int>(configuration.thruster_configs.size());
    Eigen::VectorXd limits_min(n), limits_max(n);
    for (int i = 0; i < n; ++i)
    {
        limits_min(i) = configuration_.thruster_configs[i].min_pwm;
        limits_max(i) = configuration_.thruster_configs[i].max_pwm;
    }
    joint_limits_ = {limits_min, limits_max};
}

RobotDriverBrov2::~RobotDriverBrov2()
{

}

Eigen::VectorXd RobotDriverBrov2::get_joint_positions()
{
    // there is no true sensor feedback, it just returns the last commanded PWM vector (command echo)
    if (!impl_->connected)
        throw std::runtime_error("get_joint_positions() called before connect().");

    if (impl_->last_pwm_.size() == 0)
        throw std::runtime_error("get_joint_positions(): commanded PWM not initialized yet.");

    return impl_->last_pwm_;
}

void RobotDriverBrov2::set_target_joint_positions(const Eigen::VectorXd&)
{
    throw std::runtime_error(
        "RobotDriverBrov2: position interface is not supported. Use set_target_joint_torques().");
}

/**
 * @brief RobotDriverBrov2::get_joint_torques
 * Returns the last commanded wrench. This method throws an exception
 * if called in an invalid state (not connected, not initialized, or not yet commanded).
 *
 * @return an Eigen::VectorXd with the last commanded 6D wrench.
 */
Eigen::VectorXd RobotDriverBrov2::get_joint_torques()
{
    // there is no true sensor feedback, it just returns the last commanded PWM vector (command echo)
    if (!impl_->connected)
        throw std::runtime_error("get_joint_torques() called before connect().");

    if (!impl_->initialized)
        throw std::runtime_error("get_joint_torques() called before initialize().");

    if (impl_->last_tau_.size() != 6)
        throw std::runtime_error("get_joint_torques(): commanded tau not initialized yet.");

    return impl_->last_tau_;
}

/**
 * @brief RobotDriverBrov2::set_target_joint_torques
 * Allocates the desired 6D wrench to the thrusters, converts the thruster
 * forces to PWM, and publishes them as an RC override.
 *
 * @param tau The desired 6D wrench.
 */
void RobotDriverBrov2::set_target_joint_torques(const Eigen::VectorXd &tau) {  //  receives  PWM commands as a vector of doubles.

    if (!impl_->connected || !impl_->initialized)
        throw std::runtime_error("Not connected/initialized.");

    // 1 - Input check: tau must be 6D
    if (tau.size() !=6)
    {
        throw std::runtime_error("Incorrect vector size in RobotDriverBrov2::set_target_joint_torques");
    }
	impl_->last_tau_ = tau;

    // 2 - Check publisher exists
    if (!impl_->rt_override_rc_pub) {
        throw std::runtime_error("RC override publisher not ready.");
	}

	// 3 - Allocation
	const auto N = static_cast<Eigen::Index>(configuration_.thruster_configs.size());
	if (impl_->B_pinv.rows() != N || impl_->B_pinv.cols() != 6)
		throw std::runtime_error("Allocation matrix pseudoinverse not initialized or wrong size");

	Eigen::VectorXd f = impl_->B_pinv * tau;  // thruster force

    // 4 -  Publish pwm overrides to corresponding RC channel
    if (impl_->rt_override_rc_pub && impl_->rt_override_rc_pub->trylock()) { //checking the publisher object exists and if we can get access
        impl_->rt_override_rc_pub->msg_.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE); // don't override unset channels
        for (int i = 0; i < static_cast<int>(configuration_.thruster_configs.size()); ++i) {  //number of thrusters confgured
            const auto & cfg = configuration_.thruster_configs[i];  // makes cfg refer to the configuration data for thruster i

			const double Fi = std::clamp(f(i), cfg.f_min, cfg.f_max);

            double pwm;

            if (configuration_.pwm_mapping == "linear")
            {
                pwm = force_to_pwm_linear(
                      Fi,
                      cfg.min_pwm,
                      cfg.max_pwm
                  );
            }
            else if (configuration_.pwm_mapping == "bivariate")
            {
                pwm = force_to_pwm_bivariate(
                    Fi,
                    configuration_.v_bus,
                    configuration_.fwd_thrust_curve_coeffs,
                    configuration_.rev_thrust_curve_coeffs,
                    cfg.neutral_pwm,
                    cfg.deadband_pwm,
                    cfg.min_pwm,
                    cfg.max_pwm
                );
            }
            else
            {
                throw std::invalid_argument("Unsupported PWM mapping");
            }

            impl_->rt_override_rc_pub->msg_.channels[cfg.channel - 1] = static_cast<uint16_t>(std::lround(pwm)); // write value into correct RC channel slot
            impl_->last_pwm_(i) = pwm;   // stores the last commanded PWM per thruster for logging
        }
        impl_->rt_override_rc_pub->unlockAndPublish();  //actually publish it, Publishes the current msg_ on /mavros/rc/override
    }
}

/**
 * @brief RobotDriverBrov2::connect
 *
 * Usually this method will connect to a given ip address. It is also common
 * for this part of the code to stop running programs or turn the robot off.
 * This function is expected to throw an exception of something goes wrong.
 * For instance, if the connection is not established an exception MUST
 * be thrown.
 */
void RobotDriverBrov2::connect()
{
    //An example of exception to throw.
    if(impl_->connected)
        throw std::runtime_error("Already connected.");

    // 1 - Create a node
    impl_->node = rclcpp::Node::make_shared("sas_thruster_node");

    // 2-  Create the RC override publisher + wrap it in a realtime safe publisher
    impl_->override_rc_pub =
        impl_->node->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            configuration_.rc_override_topic,
            rclcpp::SystemDefaultsQoS());

    impl_->rt_override_rc_pub =
    std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(
        impl_->override_rc_pub);

    // 3 - initialize all RC channels to not touch uncontrolled channels
    impl_->rt_override_rc_pub->msg_.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);

    // 4 - Create service client to mavros parameter service
    impl_->set_params_client = impl_->node->create_client<rcl_interfaces::srv::SetParameters>(configuration_.param_service_name);  // "/mavros/param/set_parameters"

    RCLCPP_INFO(impl_->node->get_logger(),
    "Waiting for %s...",
    configuration_.param_service_name.c_str());

    while (!(*break_loops_) && !impl_->set_params_client->wait_for_service(
        std::chrono::milliseconds(50)))
    {

    }

    if (*break_loops_)
        throw std::runtime_error("Interrupted while waiting for MAVROS param service.");

    if (!impl_->set_params_client->service_is_ready())
        throw std::runtime_error("MAVROS param service not available.");

    // Check MAVROS connection to ArduSub
    impl_->mavros_state_sub = impl_->node->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            rclcpp::SensorDataQoS(),
            [this](mavros_msgs::msg::State::SharedPtr msg) {
                impl_->mavros_connected = msg->connected;
            });

    // Wait for MAVROS connection
    const auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !(*break_loops_) && !impl_->mavros_connected &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(10))
    {
        rclcpp::spin_some(impl_->node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!impl_->mavros_connected)
        throw std::runtime_error("MAVROS is not connected to ArduSub.");

    RCLCPP_INFO(impl_->node->get_logger(), "MAVROS connected to ArduSub.");

    impl_->connected = true;
    impl_->motor_on = false;
    RCLCPP_INFO(impl_->node->get_logger(),
    "RobotDriverThruster: connected, publishing RC overrides on '%s'.",
    configuration_.rc_override_topic.c_str());

    //service exists, not called yet
}

/**
 * @brief RobotDriverBrov2::initialize
 *
 * This method is expected to turn the robot on and initialize the internal joint control loop.
 * If there are any issues, this MUST throw an exception so that the program will finish
 * cleanly.
 *
 * After this method finishes target joint states can be received.
 */
void RobotDriverBrov2::initialize()
{
     if (!impl_->connected)
        throw std::runtime_error("Not connected.");

    // Call the service
    while (!(*break_loops_))
    {
        // 1 - Build the request
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        for (const auto & cfg : configuration_.thruster_configs) {
            rcl_interfaces::msg::Parameter param;    // ROS2 request message type of whole object (parameter change)
            param.name = cfg.servo_function_param;   // Field 1 name: "SERVO1_FUNCTION": which ArduSub parameter to change
            param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER; // type of value
            param.value.integer_value = cfg.servo_function_values;           // Field 2 value: change to 1 means RC passthrough.
            request->parameters.push_back(param);    // Put that instruction into request
        }

		if (configuration_.set_battery_voltage)
		{
  	       rcl_interfaces::msg::Parameter batt_param;
   		   batt_param.name = configuration_.sim_batt_voltage_param;
           batt_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
           batt_param.value.double_value = configuration_.v_bus;
           request->parameters.push_back(batt_param);
		}

         // 2 - Send request
        auto future = impl_->set_params_client->async_send_request(request);

        // 3 - Wait for response
        if (rclcpp::spin_until_future_complete(
                impl_->node, future,
                std::chrono::seconds(5)) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(impl_->node->get_logger(),
                "All thrusters set to RC passthrough.");

		// 4 - Allocation setup
		const auto N = static_cast<Eigen::Index>(configuration_.thruster_configs.size());
		if (configuration_.B.rows() != 6 || configuration_.B.cols() != N)
			throw std::runtime_error("Allocation matrix B had wrong size");

		impl_->B_pinv = configuration_.B.completeOrthogonalDecomposition().pseudoInverse();

		if (impl_->last_pwm_.size() != N){
			impl_->last_pwm_ = Eigen::VectorXd::Constant(N, 1500);
		}

	    // 4 - Chanel range check
        const auto chan_count = static_cast<int>(impl_->rt_override_rc_pub->msg_.channels.size());
   	    for (int i = 0; i < static_cast<int>(configuration_.thruster_configs.size()); ++i) {
        const auto & cfg = configuration_.thruster_configs[i];
        if (cfg.channel < 1 || cfg.channel > chan_count)
            throw std::runtime_error(
                "Thruster " + std::to_string(i) +
                " has invalid channel " + std::to_string(cfg.channel));
        }

        // 5 - Send neutral
        impl_->rt_override_rc_pub->msg_.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);

        for (int i = 0; i < static_cast<int>(configuration_.thruster_configs.size()); ++i) //number of thrusters confgured
        {
            const auto & cfg = configuration_.thruster_configs[i]; // makes cfg refer to the configuration data for thruster i
            const int idx = cfg.channel - 1;   // convert RC channel numbering to C++ array index

            if (idx < 0 || idx >= static_cast<int>(impl_->rt_override_rc_pub->msg_.channels.size()))
                throw std::runtime_error("Thruster channel out of range.");  // prevent accessing channels out of range.

            impl_->rt_override_rc_pub->msg_.channels[idx] = cfg.neutral_pwm;  // override only this thruster’s channel with its neutral PWM value
            impl_->last_pwm_(i) = cfg.neutral_pwm;   // internal bookkeeping of last commanded pwm
        }

        impl_->override_rc_pub->publish(impl_->rt_override_rc_pub->msg_);  // publishes the OverrideRCIn message to the ROS topic e.g. /mavros/rc/override
		impl_->last_tau_ = Eigen::VectorXd::Zero(6);
        impl_->motor_on = true;
        impl_->initialized = true;
        return;
        }

        RCLCPP_WARN(impl_->node->get_logger(),
            "Service call timed out, retrying...");
    }
    throw std::runtime_error("initialize() did not complete successfully (interrupted).");
}

/**
 * @brief RobotDriverBrov2::deinitialize.
 * For safety reasons, this MUST NOT throw exceptions.
 */
void RobotDriverBrov2::deinitialize()
{
    if (!impl_->connected)  // if not connected, we don’t have a valid ROS node/publisher/service client to talk to MAVROS
        return;

    if (!impl_->initialized) // If never initialized, then there’s nothing to shut down
        return;

    //  Send neutral
    impl_->rt_override_rc_pub->msg_.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE); // override nothing

    for (int i = 0; i < static_cast<int>(configuration_.thruster_configs.size()); ++i)
    {
        const auto & cfg = configuration_.thruster_configs[i];
        const int idx = cfg.channel - 1;

        if (idx < 0 || idx >= static_cast<int>(impl_->rt_override_rc_pub->msg_.channels.size()))
            throw std::runtime_error("Thruster channel out of range.");

        impl_->rt_override_rc_pub->msg_.channels[idx] = cfg.neutral_pwm;
        impl_->last_pwm_(i) = cfg.neutral_pwm;
    }

    impl_->override_rc_pub->publish(impl_->rt_override_rc_pub->msg_);
    impl_->motor_on = false;
    impl_->initialized = false;

    // Q: should it restore the ardusub parameters? the passthrough change?
}

/**
 * @brief RobotDriverBrov2::disconnect
 * For safety reasons, this MUST NOT throw exceptions.
 */
void RobotDriverBrov2::disconnect()
{
    if (!impl_->connected) {
        return;
    }

    impl_->connected = false;
    impl_->rt_override_rc_pub.reset();
    impl_->override_rc_pub.reset();
    impl_->mavros_state_sub.reset();
    impl_->mavros_connected = false;
    impl_->node.reset();
}

}
