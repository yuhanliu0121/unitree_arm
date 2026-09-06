#!/usr/bin/env zsh

set -eo pipefail

workspace=${0:A:h:h:h:h}
source ${workspace}/scripts/lib/environment.zsh
d1_resolve_conda_executable || exit 1
conda_bin=${REPLY}
d1_resolve_ros_setup || exit 1
ros_setup=${REPLY}
headless=false
rviz=false
stage=compute

function usage() {
  print 'Usage: ./accept_visual_cube_grasp.zsh [--headless] [--rviz] [--stage compute|pregrasp|descend|lift|carry]'
}

while (( $# > 0 )); do
  case "$1" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    --stage) shift; stage=${1:-} ;;
    -h|--help) usage; exit 0 ;;
    *) print -u2 "Unknown option: $1"; usage >&2; exit 2 ;;
  esac
  shift
done
if [[ ${stage} != compute && ${stage} != pregrasp && ${stage} != descend && ${stage} != lift && ${stage} != carry ]]; then
  print -u2 "Invalid stage: ${stage}"; usage >&2; exit 2
fi

source ${ros_setup}
source ${workspace}/install/setup.zsh
set -u
export PYTHONPATH="${workspace}/simulation/d1_mujoco_sim/src${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/d1_ros_logs}
mkdir -p ${ROS_LOG_DIR}
run_dir=$(mktemp -d /tmp/d1_visual_cube.XXXXXX)
sim_log=${run_dir}/mujoco.log
stack_log=${run_dir}/stack.log
goal_log=${run_dir}/goal.log
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

sim_args=(${conda_bin} run --no-capture-output -n trash_collection python -m d1_mujoco_sim.cli --ros-camera --ros-scene --fixed-layout)
[[ ${headless} == true ]] && sim_args+=(--headless)

print '[1/3] Starting MuJoCo RGB-D scene...'
cd ${workspace}
setsid ${sim_args[@]} >${sim_log} 2>&1 &
sim_pid=$!

sim_ready=false
for attempt in {1..120}; do
  kill -0 ${sim_pid} 2>/dev/null || { tail -n 60 ${sim_log} >&2; exit 1; }
  if grep -q 'D1 MuJoCo service ready' ${sim_log}; then
    sim_ready=true
    break
  fi
  sleep 0.5
done
[[ ${sim_ready} == true ]] || {
  print -u2 'MuJoCo DDS readiness timeout'
  tail -n 60 ${sim_log} >&2
  exit 1
}

print '[2/3] Starting control, MoveIt, perception, and PickObject...'
setsid ${conda_bin} run --no-capture-output -n trash_collection ros2 launch d1_manipulation observe_target.launch.py launch_rviz:=${rviz} backend:=simulation enable_commissioning_api:=true >${stack_log} 2>&1 &
stack_pid=$!

ready=false
for attempt in {1..160}; do
  kill -0 ${sim_pid} 2>/dev/null || { tail -n 60 ${sim_log} >&2; exit 1; }
  kill -0 ${stack_pid} 2>/dev/null || { tail -n 100 ${stack_log} >&2; exit 1; }
  if grep -q 'PickObject action server ready' ${stack_log} && grep -q 'DetectTarget ready' ${stack_log} && grep -q 'You can start planning now' ${stack_log} && grep -Eq 'Configured and activated.*arm_controller' ${stack_log}; then
    ready=true; break
  fi
  sleep 0.5
done
[[ ${ready} == true ]] || { print -u2 'Stack readiness timeout'; tail -n 120 ${stack_log} >&2; exit 1; }

if [[ ${rviz} == true && -t 0 ]]; then
  print -n 'Initial scene ready. Press Enter to run staged visual cube grasp...'
  IFS= read -r reply
fi
print "[3/3] Running visual cube stage: ${stage}"
set +e
${conda_bin} run --no-capture-output -n trash_collection \
  ros2 run d1_manipulation pick_seed_cube --stage ${stage} 2>&1 | tee ${goal_log}
goal_status=${pipestatus[1]}
set -e
if (( goal_status == 0 )) && grep -q 'PICK STAGE SUCCEEDED' ${goal_log}; then
  if [[ -d /tmp/d1_cube_debug_latest ]]; then
    mkdir -p ${run_dir}/geometry
    cp -a /tmp/d1_cube_debug_latest/. ${run_dir}/geometry/
  fi
  print
  print "ACCEPTANCE PASSED: visual cube stage ${stage}"
  print "Logs: ${run_dir}"
  if [[ ${rviz} == true && -t 0 ]]; then
    print -n 'Inspect the held result and debug layers, then press Enter to close...'
    IFS= read -r reply
  fi
  exit 0
fi
print -u2 'ACCEPTANCE FAILED. Diagnostic summary:'
grep -E 'FAILED|error|Error|RANSAC|timed out|unavailable|ABORTED' ${goal_log} ${stack_log} 2>/dev/null | tail -n 30 >&2 || true
print -u2 "Full logs: ${run_dir}"
exit 1
