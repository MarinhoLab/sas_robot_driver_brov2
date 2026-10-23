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


#include "sas_robot_driver_brov2/driver_thruster_mapping.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sas
{
    double linear_poly(double F)
    {
       return 1500.0 + 8.0 * F;
    }

    double bivariate_poly(double F, double V, const std::vector<double>& c)
    {
        return c[0] + c[1]*F + c[2]*V + c[3]*F*F + c[4]*F*V;
    }

    double force_to_pwm_linear(double F, double min_pwm, double max_pwm)
    {
        double pwm = linear_poly(F);

        return std::clamp(pwm, min_pwm, max_pwm);
    }

    double force_to_pwm_bivariate(double F, double V,
                                 const std::vector<double>& fwd,
                                 const std::vector<double>& rev,
                                 int neutral_pwm, int deadband_pwm,
                                 double min_pwm, double max_pwm)
    {
        const double force_deadband_fwd_N = 0.40;
        const double force_deadband_rev_N = 0.31;
        const double force_zero_eps = 1e-9;

        if (std::abs(F) <= force_zero_eps)
        {
            return static_cast<double>(neutral_pwm);
        }

        if (F > 0.0 && F < force_deadband_fwd_N)
            return static_cast<double>(neutral_pwm);

        if (F < 0.0 && -F < force_deadband_rev_N)
            return static_cast<double>(neutral_pwm);

        double pwm = (F > 0.0) ? bivariate_poly(F, V, fwd)
                             : bivariate_poly(F, V, rev);

        if (std::abs(pwm - neutral_pwm) <= deadband_pwm)
            pwm = neutral_pwm;

        return std::clamp(pwm, min_pwm, max_pwm);
    }
}

