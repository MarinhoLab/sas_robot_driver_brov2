# Agent notes

## Re-creating the simulation video

Records the onboard camera of the simulated BlueROV2 (Blue, headless) while `scripts/driver_wrench_example.py`
drives it through the SAS driver. Validated on Apple Silicon with Docker Desktop, where Blue runs emulated (amd64),
so the sim only publishes about 5 fps.

### 1. Start Blue headless

```bash
docker pull --platform linux/amd64 ghcr.io/robotic-decision-making-lab/blue:jazzy-desktop
docker run -d --name blue-sim --platform linux/amd64 --network host --ipc host \
  -e ROS_DOMAIN_ID=0 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ghcr.io/robotic-decision-making-lab/blue:jazzy-desktop tail -f /dev/null

# Run Gazebo as a server only, with software rendering (no GUI/X server needed)
docker exec blue-sim bash -lc 'sed -i "s|-v 4 -r|-s --headless-rendering -v 3 -r|" \
  /home/ubuntu/ws_blue/install/ardusub_bringup/share/ardusub_bringup/launch/sitl.launch.yaml'

# The PATH/plugin variables are only set in ~/.bashrc, so export them for non-interactive shells
docker exec -d blue-sim bash -lc '
  export HOME=/home/ubuntu
  export PATH=$HOME/ardupilot/build/sitl/bin:$PATH
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
  export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds
  source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ws_blue/install/setup.bash
  env -u LIBGL_ALWAYS_SOFTWARE GZ_IP=127.0.0.1 ros2 launch blue_bringup bluerov2.launch.yaml use_sim:=true > /tmp/blue.log 2>&1'
```

Wait about 2 minutes, then check that `ros2 topic echo /mavros/state --once` (inside `blue-sim`) shows `connected: true`.

> [!WARNING]
> Gazebo downloads the `Sun` and `Coast Water` models from Fuel at startup. If `/tmp/blue.log` shows
> `REST response code: 502` (or `Unable to find uri`), Fuel was down. Run `docker restart blue-sim` and launch again.

### 2. Start the SAS driver and arm the vehicle

```bash
cd docker
docker compose up -d
docker compose exec -d sas bash -lc 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; \
  ros2 launch sas_robot_driver_brov2 brov2_launch.py > /tmp/sas.log 2>&1'

# The vehicle starts disarmed; it ignores RC overrides until armed
docker exec blue-sim bash -lc 'source /opt/ros/jazzy/setup.bash; \
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"'
```

### 3. Record the camera

Save the following as `record.py` and copy it into `blue-sim` with `docker cp`. It subscribes to `/camera/image_raw`
and writes an mp4 using OpenCV (already in the Blue image). Usage: `python3 record.py <out.mp4> <seconds>`.

```python
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

out, duration = sys.argv[1], float(sys.argv[2])


class Rec(Node):
    def __init__(self):
        super().__init__('rec')
        self.w = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 10, (960, 540))
        self.n = 0
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

    def cb(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding.startswith('rgb'):
            a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        self.w.write(cv2.resize(a, (960, 540)))
        self.n += 1


rclpy.init()
n = Rec()
t0 = time.time()
while time.time() - t0 < duration:
    rclpy.spin_once(n, timeout_sec=0.1)
n.w.release()
print('frames', n.n)
```

Start the recorder, then run the wrench test (it takes about two minutes).
The script connects to `brov2_1` by default, so it can be run directly (pass `brov2_heavy_1` for the heavy launch):

```bash
docker exec -d blue-sim bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /tmp/record.py /tmp/wrench.mp4 125 > /tmp/rec.log 2>&1'

cd docker
docker compose exec -T sas bash -lc 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; \
  ros2 run sas_robot_driver_brov2 driver_wrench_example.py'
```

The `.mp4` is only finalized when the recorder exits, so wait for `/tmp/rec.log` to print `frames <N>`
before copying it out with `docker cp blue-sim:/tmp/wrench.mp4 .`.

### 4. Encode

The sim runs slower than real time, so the frames are retimed to the achieved frame rate (`N` frames over the
recording duration, e.g. 655 frames over 125 s is 5.24 fps), then re-encoded as H.264 with the host `ffmpeg`:

```bash
ffmpeg -y -i wrench.mp4 -vf "setpts=PTS*10/<fps>" -r 10 -c:v libx264 -pix_fmt yuv420p -crf 26 brov2_wrench_sim.mp4
```

Check a few frames (for example with `ffmpeg -ss 5 -i brov2_wrench_sim.mp4 -frames:v 1 f.png`) to make sure the
scene is rendered and changes over time. The camera sits at the water surface, so the view is sky and seabed.

### 5. Clean up

```bash
(cd docker && docker compose down)
docker rm -f blue-sim
```

Do not commit the video to the repository.
