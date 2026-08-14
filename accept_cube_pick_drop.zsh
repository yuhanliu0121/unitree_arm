#!/usr/bin/env zsh

set -eo pipefail

workspace=${0:A:h}
conda_bin=/home/tony/miniconda3/bin/conda
headless=false
rviz=false
object=cube

function usage() {
  print 'Usage: ./accept_cube_pick_drop.zsh [--headless] [--rviz] [--object cube|zucchini|bowl]'
}
while (( $# > 0 )); do
  case "$1" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    --object) shift; object=${1:-} ;;
    -h|--help) usage; exit 0 ;;
    *) print -u2 "Unknown option: $1"; usage >&2; exit 2 ;;
  esac
  shift
done
if [[ ${object} != cube && ${object} != zucchini && ${object} != bowl ]]; then
  print -u2 "Invalid object: ${object}"; usage >&2; exit 2
fi

if [[ ${object} == bowl ]]; then
  class_name=bowl
  pick_client=pick_seed_bowl
elif [[ ${object} == zucchini ]]; then
  class_name=zucchini
  pick_client=pick_seed_zucchini
else
  class_name=yellow_cube
  pick_client=pick_seed_cube
fi

source /opt/ros/humble/setup.zsh
source ${workspace}/install/setup.zsh
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/d1_ros_logs}
mkdir -p ${ROS_LOG_DIR}
run_dir=$(mktemp -d /tmp/d1_pick_drop.XXXXXX)
sim_log=${run_dir}/mujoco.log
stack_log=${run_dir}/stack.log
pick_log=${run_dir}/pick.log
drop_log=${run_dir}/drop.log
sim_pid=''; stack_pid=''

function stop_group() {
  local pid=$1
  if [[ -n ${pid} ]] && kill -0 ${pid} 2>/dev/null; then
    /bin/kill -TERM -- -${pid} 2>/dev/null || true
    for attempt in {1..30}; do
      kill -0 ${pid} 2>/dev/null || break
      sleep 0.1
    done
    kill -0 ${pid} 2>/dev/null && /bin/kill -KILL -- -${pid} 2>/dev/null || true
    wait ${pid} 2>/dev/null || true
  fi
}
function cleanup() { trap - EXIT INT TERM; stop_group ${stack_pid}; stop_group ${sim_pid}; }
trap cleanup EXIT
trap 'exit 130' INT TERM

sim_args=(${conda_bin} run --no-capture-output -n trash_collection d1-mujoco-sim --ros-camera --ros-scene --fixed-layout)
[[ ${headless} == true ]] && sim_args+=(--headless)
[[ ${rviz} == true ]] && sim_args+=(--camera-debug)

print '[1/4] Starting deterministic MuJoCo acceptance scene...'
cd ${workspace}
setsid ${sim_args[@]} >${sim_log} 2>&1 &
sim_pid=$!
print '[2/4] Starting control, MoveIt, perception, PickObject and DropObject...'
setsid ${conda_bin} run --no-capture-output -n trash_collection ros2 launch d1_manipulation observe_target.launch.py launch_rviz:=${rviz} >${stack_log} 2>&1 &
stack_pid=$!

ready=false
for attempt in {1..180}; do
  kill -0 ${sim_pid} 2>/dev/null || { tail -n 80 ${sim_log} >&2; exit 1; }
  kill -0 ${stack_pid} 2>/dev/null || { tail -n 120 ${stack_log} >&2; exit 1; }
  if grep -q 'PickObject action server ready' ${stack_log} && \
     grep -q 'DropObject action server ready' ${stack_log} && \
     grep -q 'DetectTarget ready' ${stack_log} && \
     grep -q 'You can start planning now' ${stack_log} && \
     grep -Eq 'Configured and activated.*arm_controller' ${stack_log}; then
    ready=true; break
  fi
  sleep 0.5
done
[[ ${ready} == true ]] || { print -u2 'Stack readiness timeout'; tail -n 140 ${stack_log} >&2; exit 1; }

if [[ ${rviz} == true && -t 0 ]]; then
  print -n "Scene ready. Press Enter to pick ${class_name} and move to CARRY..."
  IFS= read -r reply
fi
print "[3/4] Picking ${class_name} into CARRY..."
set +e
ros2 run d1_manipulation ${pick_client} --stage carry 2>&1 | tee ${pick_log}
pick_status=${pipestatus[1]}
set -e
if (( pick_status != 0 )) || ! grep -q 'PICK STAGE SUCCEEDED' ${pick_log}; then
  print -u2 'PICK FAILED. Diagnostic summary:'
  grep -E 'FAILED|error|Error|timed out|unavailable|ABORTED' ${pick_log} ${stack_log} 2>/dev/null | tail -n 30 >&2 || true
  print -u2 "Full logs: ${run_dir}"
  exit 1
fi

if [[ ${rviz} == true && -t 0 ]]; then
  print -n 'CARRY reached. Inspect it, then press Enter to release into the bin...'
  IFS= read -r reply
fi
print "[4/4] Releasing ${class_name} into the acceptance trash bin..."
set +e
ros2 run d1_manipulation drop_seed_bin --object ${class_name} 2>&1 | tee ${drop_log}
drop_status=${pipestatus[1]}
set -e
if (( drop_status != 0 )) || ! grep -q 'DROP SUCCEEDED' ${drop_log}; then
  print -u2 'DROP FAILED. Diagnostic summary:'
  grep -E 'FAILED|error|Error|timed out|unavailable|ABORTED' ${drop_log} ${stack_log} 2>/dev/null | tail -n 30 >&2 || true
  print -u2 "Full logs: ${run_dir}"
  exit 1
fi

print
print "ACCEPTANCE PASSED: ${class_name} picked, carried, released into bin, and arm stowed."
print "Logs: ${run_dir}"
if [[ ${rviz} == true && -t 0 ]]; then
  print -n 'Inspect the final scene, then press Enter to close...'
  IFS= read -r reply
fi
