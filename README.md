# sas_robot_driver_brov2

> [!TIP]
> More information about the SmartArmStack is available in https://smartarmstack.github.io/.

The SAS driver can be tested with the [Blue](https://github.com/Robotic-Decision-Making-Lab/blue) simulation environment for the BlueROV2 and BlueROV2 Heavy.

> [!IMPORTANT]
> Use the same ROS 2 distribution for SAS and Blue (Jazzy).
> The containers run separately and communicate through ROS 2, so they must also
> use the same `ROS_DOMAIN_ID` (default: `0`) and host networking
> (`network_mode: host`).
 
### 1. Launch the simulation

Follow the [Blue documentation](https://robotic-decision-making-lab.github.io/blue/) to set up and start the appropriate Blue container.
 
Inside the Blue container, launch the desired vehicle:
 
```bash
env -u LIBGL_ALWAYS_SOFTWARE \
    GZ_IP=127.0.0.1 \
    ros2 launch blue_bringup <model>.launch.yaml use_sim:=true
```
 
Replace `<model>` with `bluerov2` or `bluerov2_heavy`.
 
### 2. Launch the SAS driver

From the directory containing the SAS Docker Compose file, start the SAS container:

```bash
docker compose up -d --build
```
Enter the SAS container: 

```bash
docker compose exec sas bash
```

Then launch the driver for the simulated vehicle:

```bash
ros2 launch sas_robot_driver_brov2 <model>_launch.py
```

Replace `<model>` with `brov2` or `brov2_heavy`.

> [!NOTE]
> MAVROS provides the ROS 2 interface to ArduSub. To verify that this communication is active, run:
>
> ```bash
> ros2 topic echo /mavros/state --once
> ```

 
### 3. Test the driver
 
Run the wrench testing script:
 
```bash
ros2 run sas_robot_driver_brov2 driver_wrench_example.py
```

The script connects to the driver node `brov2_1` (started by `brov2_launch.py`) by default.
For the heavy vehicle, pass the node name of `brov2_heavy_launch.py`:

```bash
ros2 run sas_robot_driver_brov2 driver_wrench_example.py brov2_heavy_1
```

## ROS 2 Nodes & Parameters

### Node: `sas_robot_driver_brov2_node`

| Property        | Value                                                                                                                                                                                                             |
|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Executable**  | `sas_robot_driver_brov2_node`                                                                                                                                                                                     |
| **Description** | Reads all parameters, instantiates `sas::RobotDriverBrov2` and runs it with `sas::RobotDriverROS`. A 6D wrench set as the joint torques is allocated to the thrusters and sent as PWM through MAVROS RC override. |

#### Parameters

All parameters are **mandatory** and read with `sas::get_ros_parameter(...)`. If one is missing, the node throws and fails to start.
The per-thruster arrays must all have the same length, equal to the number of thrusters.

| Parameter                                               | Type               | Purpose                                                                 |
|---------------------------------------------------------|--------------------|-------------------------------------------------------------------------|
| `rc_override_topic`                                     | string             | Topic to which the RC override is published (e.g. `mavros/rc/override`) |
| `param_service_name`                                    | string             | MAVROS parameter service (e.g. `/mavros/param/set_parameters`)          |
| `batt_voltage`                                          | double             | Bus (battery) voltage                                                   |
| `thread_sampling_time_sec`                              | double             | Sampling period of the control-loop thread                              |
| `thruster_channels`                                     | array of int       | RC channel (1-based) of each thruster                                   |
| `thruster_neutral_pwm`                                  | array of int       | PWM sent when each thruster is stopped                                  |
| `thruster_min_pwm` / `thruster_max_pwm`                 | array of double    | PWM limits of each thruster                                             |
| `thruster_min_force` / `thruster_max_force`             | array of double    | Force limits of each thruster                                           |
| `thruster_deadband_pwm`                                 | array of double    | PWM deadband of each thruster                                           |
| `servo_function_params`                                 | array of string    | ArduSub parameter of each thruster (e.g. `SERVO1_FUNCTION`)             |
| `servo_function_values`                                 | array of int       | Value set for each servo function (`1` is RC passthrough)               |
| `fwd_thrust_curve_coeffs` / `rev_thrust_curve_coeffs`   | array of 5 doubles | Thrust curve coefficients                                               |
| `tam.x`, `tam.y`, `tam.z`, `tam.rx`, `tam.ry`, `tam.rz` | array of double    | Rows of the 6 x N thruster allocation matrix                            |
