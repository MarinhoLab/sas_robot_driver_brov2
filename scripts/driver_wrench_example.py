#!/usr/bin/python3
"""
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
# #######################################################################################
#
#   Author: Rawan M. Elsayed, email: rawan.elsayed@manchester.ac.uk
#   Co-author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# #######################################################################################
"""

import time

import numpy
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver import RobotDriverClient

from sas_core import Clock, Statistics

# 1. Test configuration
dt = 0.01
test_duration = 3.0
pause_duration = 4.0

force = 5.0             # Translation
moment = 0.5            # Rotation

tau_zero = numpy.zeros(6, dtype=numpy.float64)


# 2. Test scenarios: wrench = [X, Y, Z, K, M, N]
S_driver = numpy.diag([1, -1, -1, 1, -1, -1])

basic_translations = [
    ("Forward",  [ force, 0, 0, 0, 0, 0]),
    ("Backward", [-force, 0, 0, 0, 0, 0]),
    ("Left",     [0,  force, 0, 0, 0, 0]),
    ("Right",    [0, -force, 0, 0, 0, 0]),
    ("Up",       [0, 0,  force, 0, 0, 0]),
    ("Down",     [0, 0, -force, 0, 0, 0]),
]

basic_rotations = [
    ("Roll positive",  [0, 0, 0,  moment, 0, 0]),
    ("Roll negative",  [0, 0, 0, -moment, 0, 0]),
    ("Pitch positive", [0, 0, 0, 0,  moment, 0]),
    ("Pitch negative", [0, 0, 0, 0, -moment, 0]),
    ("Yaw positive",   [0, 0, 0, 0, 0,  moment]),
    ("Yaw negative",   [0, 0, 0, 0, 0, -moment]),
]

combined_motions = [
    ("Forward + Left",  [force, force, 0, 0, 0, 0]),
    ("Forward + Yaw",   [force, 0, 0, 0, 0, moment]),
    ("Forward + Pitch", [force, 0, 0, 0, moment, 0]),
]

# 3. Send wrench and execute a test group
def run_motion(rdi, node, clock, name, tau, duration):
    target_joint_forces = numpy.array(tau, dtype=numpy.float64)

    print(f"\nRunning: {name}")
    print(f"Target joint forces: {target_joint_forces}")

    steps = int(duration / dt)

    for i in range(steps):
        clock.update_and_sleep()

        rdi.send_target_joint_forces(S_driver @ target_joint_forces)
        rclcpp_spin_some(node)

def run_test_group(rdi, node, clock, group_name, tests):
    print(f"\n========== {group_name} ==========")

    for name, tau in tests:
        run_motion(rdi, node, clock, name, tau, test_duration)
        run_motion(rdi, node, clock, "Neutral", tau_zero, pause_duration)

def main(args=None):
    rdi = None
    node = None
    ros_initialized = False

    try:
        rclcpp_init()
        ros_initialized = True

        node = rclcpp_Node("sas_robot_driver_brov2_driver_wrench_example_node_cpp")

        # 10 ms clock
        clock = Clock(dt)
        clock.init()

        # Initialize the RobotDriverClient
        rdi = RobotDriverClient(node, 'brov2_heavy_1')

        # Wait for RobotDriverClient to be enabled
        while not rdi.is_enabled():
            rclcpp_spin_some(node)
            time.sleep(0.1)

        # Get topic information
        print(f"topic prefix = {rdi.get_topic_prefix()}")

        # Read the values sent by the RobotDriverServer
        tau = rdi.get_joint_forces()
        print(f"tau = {tau}")

        # Execute motion tests
        #run_test_group(rdi, node, clock, "Basic translations", basic_translations)
        run_test_group(rdi, node, clock, "Basic rotations", basic_rotations)
        #run_test_group(rdi, node, clock, "Combined motions", combined_motions)

        # Statistics
        print("Statistics for the entire loop")
        print("  Mean computation time: {}".format(clock.get_statistics(
            Statistics.Mean, Clock.TimeType.Computational)
        ))
        print("  Mean idle time: {}".format(clock.get_statistics(
            Statistics.Mean, Clock.TimeType.Idle)
        ))
        print("  Mean effective thread sampling time: {}".format(clock.get_statistics(
            Statistics.Mean, Clock.TimeType.EffectiveSampling)
        ))

    except KeyboardInterrupt:
        print("Interrupted by user")

    except Exception as e:
        print("Unhandled exception:", e)

    finally:
        # Send zero wrench before shutdown
        if rdi is not None and node is not None:
            try:
                if rdi.is_enabled():
                    for _ in range(100):
                        rdi.send_target_joint_forces(tau_zero)
                        rclcpp_spin_some(node)
                        time.sleep(dt)

                    print("Final zero wrench sent.")

            except Exception as e:
                print("Failed to send zero wrench:", e)

        if ros_initialized:
            rclcpp_shutdown()


if __name__ == '__main__':
    main()