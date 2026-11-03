#!/bin/bash
# Headless MAVROS integration test: fake_fcu.py + mavros + driver + wrench
# test, no Blue/Gazebo needed. Run inside the SAS container (docker compose
# up -d && docker compose exec sas bash) from the package root:
#   bash .agents/test_mavros.sh [node_name]   (default: brov2_1)
#
# The driver starts alongside MAVROS on purpose: MAVROS refuses parameters
# until it has fetched the FCU parameter list (~10 s after connecting), so a
# correct driver must retry param/set_parameters instead of assuming success.
# Installs ros-jazzy-mavros + pymavlink + the GeographicLib dataset if
# missing (needs network). Exit codes: 0 pass, 1 test failure, 2 environment.
set -u
node_name="${1:-brov2_1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
TMP=/tmp/mavros_test
mkdir -p "$TMP"

kill_all() {
    pkill -f "[f]ake_fcu.py" 2>/dev/null
    pkill -f "[m]avros_node" 2>/dev/null
    pkill -f "[s]as_robot_driver_brov2_node" 2>/dev/null
    sleep 1
}

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
    echo "[test] installing mavros + pymavlink + geoid dataset"
    apt-get update -qq >/dev/null || exit 2
    apt-get install -y -qq ros-jazzy-mavros python3-pip >/dev/null || exit 2
    python3 -m pip install -q --break-system-packages pymavlink || exit 2
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh >/dev/null 2>&1 || exit 2
fi

set +u  # the ROS setup chain reads unset vars
source /opt/ros/jazzy/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
set -u

kill_all
rm -f "$TMP"/*.jsonl "$TMP"/*.out

python3 .agents/fake_fcu.py --log "$TMP/fcu_log.jsonl" > "$TMP/fcu.out" 2>&1 &
sleep 1
ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://127.0.0.1:5760 \
    > "$TMP/mavros.out" 2>&1 &

connected=0
for _ in $(seq 1 60); do
    ros2 topic echo /mavros/state --once --field connected 2>/dev/null \
        | grep -qi true && { connected=1; break; }
    sleep 1
done
[ "$connected" = 1 ] || { echo "[test] FAIL: MAVROS never connected to the fake FCU"; kill_all; exit 2; }

model=brov2
case "$node_name" in brov2_heavy*) model=brov2_heavy;; esac
ros2 launch sas_robot_driver_brov2 "${model}_launch.py" > "$TMP/driver.out" 2>&1 &

initialized=0
for _ in $(seq 1 60); do
    grep -q "Robot initialized" "$TMP/driver.out" && { initialized=1; break; }
    sleep 1
done
[ "$initialized" = 1 ] || { echo "[test] FAIL: driver never initialized (see $TMP/driver.out)"; kill_all; exit 2; }

python3 .agents/mavros_wrench_test.py "$node_name" --fcu-log "$TMP/fcu_log.jsonl"
rc=$?

kill_all
echo "[test] logs: $TMP/{fcu,mavros,driver}.out $TMP/{fcu,wrench}_log.jsonl"
exit $rc
