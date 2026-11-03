#!/usr/bin/python3
"""End-to-end MAVROS integration test for the driver: steps the wrench
(zero -> forward -> zero) through <node>/set/target_joint_forces and
analyses the fake_fcu.py RC_CHANNELS_OVERRIDE log for the results the
driver cannot see from its side.

Run inside the SAS container after the driver reports "Robot initialized"
(see test_mavros.sh for the full orchestration):
  python3 .agents/fake_fcu.py --log /tmp/fcu_log.jsonl &
  ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://127.0.0.1:5760 &
  ros2 launch sas_robot_driver_brov2 brov2_launch.py &
  python3 .agents/mavros_wrench_test.py brov2_1 --fcu-log /tmp/fcu_log.jsonl

Checked: the SERVOx_FUNCTION PARAM_SETs reached the FCU, the command
stream runs at the configured thread rate, and the commanded PWM change
arrives at the FCU within a bounded latency of the wrench step.
Exit code: 0 all checks passed, 1 at least one failed."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NEUTRAL = 1500.0
TOL = 15.0  # PWM tolerance, below the 30-count deadband


def load_log(path):
    rows = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


class WrenchStepper(Node):
    def __init__(self, prefix, rate_hz):
        super().__init__("sas_brov2_mavros_wrench_test")
        self.pub = self.create_publisher(
            Float64MultiArray, f"/{prefix}/set/target_joint_forces", 10)
        self.period = 1.0 / rate_hz

    def segment(self, tau, duration, logf):
        logf.write(json.dumps({"ev": "segment_start", "tau": list(tau),
                               "t": time.time()}) + "\n")
        logf.flush()
        msg = Float64MultiArray()
        msg.data = [float(x) for x in tau]
        t_end = time.time() + duration
        while time.time() < t_end:
            self.pub.publish(msg)
            time.sleep(self.period)


def analyse(rows, wr, servo_count, max_latency_ms, min_rate_hz, expect_params):
    fails = []

    n_params = sum(1 for r in rows if r["ev"] == "param_set")
    if n_params < expect_params:
        fails.append(f"param_set: {n_params} < {expect_params} (SERVOx_FUNCTION never reached the FCU)")
    else:
        print(f"PASS: param_set: {n_params} parameters applied on the FCU")

    ovr = [r for r in rows if r["ev"] == "ovr"]

    def active_delta(ch):
        vals = [ch[i] for i in range(servo_count) if ch[i] not in (0, 65535)]
        return max((abs(v - NEUTRAL) for v in vals), default=0.0)

    segments = [r for r in wr if r["ev"] == "segment_start"]
    for prev, cur in zip(segments, segments[1:], strict=False):
        was_zero = max(abs(x) for x in prev["tau"]) < 1e-9
        is_zero = max(abs(x) for x in cur["tau"]) < 1e-9
        kind = "forward" if was_zero else "return-to-neutral"
        hit = next(
            (r for r in ovr
             if r["t"] >= cur["t"] and (active_delta(r["ch"]) > TOL) != is_zero),
            None)
        if hit is None:
            fails.append(f"latency[{kind}]: no matching RC_CHANNELS_OVERRIDE after the step")
            continue
        latency_ms = (hit["t"] - cur["t"]) * 1000.0
        if latency_ms > max_latency_ms:
            fails.append(f"latency[{kind}]: {latency_ms:.1f} ms > {max_latency_ms} ms")
        else:
            print(f"PASS: latency[{kind}]: {latency_ms:.1f} ms")
        # Override stream rate inside the steady part of this segment.
        seg = [r for r in ovr if cur["t"] + 1.0 <= r["t"] <= cur["t"] + 3.0]
        if len(seg) > 10:
            rate = (len(seg) - 1) / (seg[-1]["t"] - seg[0]["t"])
            if rate < min_rate_hz:
                fails.append(f"rate: {rate:.0f} Hz < {min_rate_hz} Hz (RealtimePublisher drops?)")
            else:
                print(f"PASS: rate: {rate:.0f} Hz override stream")

    return fails


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("prefix", nargs="?", default="brov2_1", help="driver node name")
    ap.add_argument("--fcu-log", default="/tmp/fcu_log.jsonl")
    ap.add_argument("--force", type=float, default=5.0, help="forward wrench magnitude [N]")
    ap.add_argument("--hold", type=float, default=5.0, help="zero-wrench hold [s]")
    ap.add_argument("--step", type=float, default=10.0, help="step hold [s]")
    ap.add_argument("--rate", type=float, default=100.0, help="publish rate [Hz]")
    ap.add_argument("--servo-count", type=int, default=6)
    ap.add_argument("--max-latency-ms", type=float, default=100.0)
    ap.add_argument("--min-rate-hz", type=float, default=300.0)
    ap.add_argument("--expect-params", type=int, default=6)
    args = ap.parse_args()

    rclpy.init(args=None)
    node = WrenchStepper(args.prefix, args.rate)
    logf = open("/tmp/wrench_log.jsonl", "w")
    zero = [0.0] * 6
    fwd = [args.force] + [0.0] * 5
    try:
        for tau, dur in ((zero, args.hold), (fwd, args.step), (zero, args.hold)):
            node.segment(tau, dur, logf)
    finally:
        logf.close()

    time.sleep(1.0)  # let the last overrides reach the FCU log
    fails = analyse(load_log(args.fcu_log), load_log("/tmp/wrench_log.jsonl"),
                    args.servo_count, args.max_latency_ms, args.min_rate_hz,
                    args.expect_params)
    for f in fails:
        print(f"FAIL: {f}")
    print("RESULT:", "FAIL" if fails else "PASS")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
