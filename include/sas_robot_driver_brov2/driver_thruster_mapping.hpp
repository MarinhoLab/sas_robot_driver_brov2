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

#pragma once

/**
 * @file driver_thruster_mapping.hpp
 * @brief Mapping functions from thruster force to PWM commands for the BlueROV2.
 */

#include <vector>

namespace sas
{
	/**
	 * @brief Evaluate a bivariate polynomial in force and bus voltage.
	 * @param F The thruster force.
	 * @param V The bus (battery) voltage.
	 * @param c The polynomial coefficients.
	 * @return The polynomial evaluated at (F, V).
	 */
	double bivariate_poly(double F, double V, const std::vector<double>& c);

	/**
	 * @brief Convert a thruster force into a PWM command.
	 * @param F The desired thruster force.
	 * @param min_pwm The minimum allowed PWM value.
	 * @param max_pwm The maximum allowed PWM value.
	 * @return The PWM command, saturated to [min_pwm, max_pwm].
	 */
	double force_to_pwm(double F, double min_pwm, double max_pwm);
}