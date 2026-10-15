# sas_robot_driver_brov2
The driver can be tested with the [Blue](https://github.com/Robotic-Decision-Making-Lab/blue) simulation environment for the BlueROV2 and BlueROV2 Heavy.
 
> [!IMPORTANT]
> Blue and the SAS driver run in separate Docker containers and communicate through ROS 2. Both containers must use the same `ROS_DOMAIN_ID` (default: `0`) and host networking (`network_mode: host`) to communicate in this setup.
 
### 1. Launch the simulation
 
Follow the [Blue documentation](https://robotic-decision-making-lab.github.io/blue/) to set up and start the Blue container.
 
Inside the Blue container, launch the desired vehicle:
 
```bash
env -u LIBGL_ALWAYS_SOFTWARE \
    GZ_IP=127.0.0.1 \
    ros2 launch blue_bringup <model>.launch.yaml use_sim:=true
```
 
Replace `<model>` with `bluerov2` or `bluerov2_heavy`.
 
### 2. Launch the SAS driver

Inside the SAS container, launch the driver corresponding to the simulated vehicle:

```bash
ros2 launch sas_robot_driver_brov2 <model>_launch.py
```

Replace `<model>` with `brov2` or `brov2_heavy`.
 
### 3. Test the driver
 
Verify that MAVROS communication is available:
 
```bash
ros2 topic echo /mavros/state --once
```
 
Run the wrench testing script:
 
```bash
ros2 run sas_robot_driver_brov2 driver_wrench_example.py
```
