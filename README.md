# sas_robot_driver_brov2
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
