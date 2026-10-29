"""Record /camera/image_raw to an mp4. Run inside the Blue container.

Usage: python3 record.py <out.mp4> <seconds>
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
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
try:
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
except Exception as e:
    print(e)
n.w.release()
print('frames', n.n)

