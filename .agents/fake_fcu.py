#!/usr/bin/python3
"""Minimal ArduSub-like MAVLink peer for testing the driver against MAVROS
without Blue/Gazebo. Run inside the SAS container, e.g.:
  python3 .agents/fake_fcu.py --log /tmp/fcu_log.jsonl &
MAVROS must connect to it: ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=tcp://127.0.0.1:5760

It sends HEARTBEAT (1 Hz) and BATTERY_STATUS (1 Hz), answers the MAVROS
parameter fetch (PARAM_REQUEST_LIST) from a small store, echoes PARAM_SET
with PARAM_VALUE, ACKs COMMAND_LONG, records every RC_CHANNELS_OVERRIDE to
a JSON-lines log, and streams SERVO_OUTPUT_RAW (50 Hz) mirroring the last
override so /mavros/rc/out reflects the commanded PWMs.

Dependencies: pip install pymavlink"""
import argparse
import json
import time

from pymavlink import mavutil

PARAM_TYPE_INT32 = 6
PARAM_TYPE_REAL = 9


class FakeFCU:
    def __init__(self, listen, log_path, battery_v, servo_count):
        self.link = mavutil.mavlink_connection(
            listen, dialect="common", source_system=1, source_component=1, quiet=True
        )
        self.log_path = log_path
        self.battery_mv = int(battery_v * 1000)
        self.servo_count = servo_count
        self.last_ovr = [65535] * 18
        self.n_ovr = 0
        self.params = {f"SERVO{i}_FUNCTION": (1, PARAM_TYPE_INT32) for i in range(1, 17)}
        self.params["BATT_VOLT_MULT"] = (18.097, PARAM_TYPE_REAL)
        self.params["SIM_BATT_VOLTAGE"] = (15.8, PARAM_TYPE_REAL)
        self.logf = open(log_path, "w")

    def log(self, event):
        event["t"] = time.time()
        self.logf.write(json.dumps(event) + "\n")
        self.logf.flush()

    def send_param_value(self, pid, val, ptype, idx, count):
        # pymavlink fills the target from the peer it heard last from.
        self.link.mav.param_value_send(pid.encode("utf-8"), float(val), ptype, count, idx)

    def send_param_list(self):
        n = len(self.params)
        for i, (pid, (val, ptype)) in enumerate(self.params.items()):
            self.send_param_value(pid, val, ptype, i, n)
        self.link.mav.param_value_send(b"", 0.0, PARAM_TYPE_REAL, n, 65535)
        self.log({"ev": "param_list_sent", "n": n})

    def wait_for_mavros(self, timeout):
        hb = self.link.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise RuntimeError("no heartbeat received from mavros")
        self.log({"ev": "connected"})

    def spin(self, duration=None):
        t_end = None if duration is None else time.time() + duration
        t_hb = t_batt = t_srv = 0.0
        while t_end is None or time.time() < t_end:
            now = time.time()
            if now - t_hb >= 1.0:
                self.link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_SUBMARINE,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    19,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                t_hb = now
            if now - t_batt >= 1.0:
                self.link.mav.battery_status_send(
                    0, 1, 2, 250,
                    [self.battery_mv] + [0] * 9, -1, -1, -1, 100, -1, 0, [0, 0, 0, 0], 0, 0,
                )
                t_batt = now
            if now - t_srv >= 0.02:
                pwm = [c if c not in (0, 65535) else 1500 for c in self.last_ovr[:8]]
                self.link.mav.servo_output_raw_send(int(now * 1e6) % 1000000, 0, *pwm)
                t_srv = now

            msg = self.link.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.001)
                continue
            self.handle(msg)

    def handle(self, msg):
        mtype = msg.get_type()
        if mtype == "RC_CHANNELS_OVERRIDE":
            ch = [getattr(msg, f"chan{i}_raw") for i in range(1, 19)]
            self.n_ovr += 1
            changed = [i for i in range(self.servo_count) if ch[i] not in (0, 65535)]
            self.log({"ev": "ovr", "n": self.n_ovr, "ch": ch[:8], "changed": changed})
            self.last_ovr = ch
        elif mtype == "PARAM_REQUEST_LIST":
            self.log({"ev": "param_request_list"})
            self.send_param_list()
        elif mtype == "PARAM_SET":
            pid = msg.param_id.split("\x00")[0]
            self.log({"ev": "param_set", "id": pid, "val": msg.param_value,
                      "type": msg.param_type})
            self.params[pid] = (msg.param_value, msg.param_type)
            self.send_param_value(pid, msg.param_value, msg.param_type, 0, 0)
        elif mtype == "COMMAND_LONG":
            self.log({"ev": "cmd", "cmd": msg.command, "p1": msg.param1})
            self.link.mav.command_ack_send(
                msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--listen", default="tcpin:0.0.0.0:5760",
                    help="pymavlink connection string to listen on (default: %(default)s)")
    ap.add_argument("--log", default="/tmp/fcu_log.jsonl", help="JSON-lines event log")
    ap.add_argument("--battery-v", type=float, default=16.0, help="reported battery voltage")
    ap.add_argument("--servo-count", type=int, default=6, help="number of thruster channels")
    args = ap.parse_args()

    fcu = FakeFCU(args.listen, args.log, args.battery_v, args.servo_count)
    print(f"[fcu] listening on {args.listen}, logging to {args.log}", flush=True)
    fcu.wait_for_mavros(timeout=120)
    print(f"[fcu] mavros online (sys={fcu.link.target_system})", flush=True)
    fcu.spin()


if __name__ == "__main__":
    main()
