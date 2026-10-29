"""
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
# #######################################################################################
#
#   Author: Rawan M. Elsayed, email: rawan.elsayed@manchester.ac.uk
#   Co-author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# #######################################################################################
"""

"""
Run this script in a different terminal window or tab. Be ready to close this,
as this activates the real robot if the connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_brov2',
            executable='sas_robot_driver_brov2_node',
            name='brov2_1',
            parameters=[{
                "batt_voltage":              15.8,
                "thread_sampling_time_sec":  0.002,
                "thruster_channels":         [1,2,3,4,5,6],
                "servo_function_values":     [1,1,1,1,1,1],
                "thruster_neutral_pwm":      [1500,1500,1500,1500,1500,1500],
                "thruster_min_pwm":          [1100.0,1100.0,1100.0,1100.0,1100.0,1100.0],
                "thruster_max_pwm":          [1900.0,1900.0,1900.0,1900.0,1900.0,1900.0],
                "thruster_max_force":        [31.25, 31.25, 31.25, 31.25, 31.25, 31.25],
                "thruster_min_force":        [-31.25, -31.25, -31.25, -31.25, -31.25, -31.25],
                "tam.x":                     [-0.707, -0.707,  0.707,  0.707,  0.0,      0.0],
                "tam.y":                     [ 0.707, -0.707,  0.707, -0.707,  0.0,      0.0],
                "tam.z":                     [ 0.0,    0.0,    0.0,    0.0,    1.0,      1.0],
                "tam.rx":                    [ 0.0,    0.0,    0.0,    0.0,   0.109, -0.109],
                "tam.ry":                    [ 0.0,    0.0,    0.0,    0.0,   0,    0],
                "tam.rz":                    [ 0.16405, -0.16405, -0.17112, 0.17112, 0.0,      0.0],
                "thruster_deadband_pwm":     [30.0,30.0,30.0,30.0,30.0,30.0],
                "fwd_thrust_curve_coeffs":   [1607.0, 14.6053, -4.2000, -0.0658, -0.2837],
                "rev_thrust_curve_coeffs":   [1402.3, 19.0531, 3.6267, 0.1016, -0.3990],
                "servo_function_params":     ["SERVO1_FUNCTION","SERVO2_FUNCTION","SERVO3_FUNCTION","SERVO4_FUNCTION",
                                              "SERVO5_FUNCTION","SERVO6_FUNCTION"],
                "rc_override_topic":          "/mavros/rc/override",
                "param_service_name":         "/mavros/param/set_parameters"
            }]
        ),

    ])
