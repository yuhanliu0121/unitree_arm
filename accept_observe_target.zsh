#!/usr/bin/env zsh

set -eo pipefail

workspace=${0:A:h}
conda_bin=/home/tony/miniconda3/bin/conda
ros_setup=/opt/ros/humble/setup.zsh
headless=false
rviz=false
server_only=false

function usage() {
  print 'Usage: ./accept_observe_target.zsh [--headless] [--rviz] [--server-only]'
  print '  --headless    Do not open the MuJoCo viewer'
  print '  --rviz        Open RViz with robot, cameras, scene, and observation markers'
  print '  --server-only Start the stack and wait for manual Action goals'
}

function wait_for_enter() {
  local prompt=$1
  if [[ ${rviz} == true && -t 0 ]]; then
    print
    print -n "${prompt}"
    local reply
    IFS= read -r reply
  fi
}

while (( $# > 0 )); do
  case "$1" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    --server-only) server_only=true ;;
    -h|--help) usage; exit 0 ;;
    *)
      print -u2 "Unknown option: $1"
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -x ${conda_bin} ]]; then
  print -u2 "Conda executable not found: ${conda_bin}"
  exit 1
fi
if [[ ! -r ${ros_setup} ]]; then
  print -u2 "ROS 2 setup not found: ${ros_setup}"
  exit 1
fi
if [[ ! -r ${workspace}/install/setup.zsh ]]; then
  print -u2 'Workspace has not been built. Run colcon build first.'
  exit 1
fi

source ${ros_setup}
source ${workspace}/install/setup.zsh
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/d1_ros_logs}
mkdir -p ${ROS_LOG_DIR}

run_dir=$(mktemp -d /tmp/d1_observe_accept.XXXXXX)
sim_log=${run_dir}/mujoco.log
stack_log=${run_dir}/observe_stack.log
goal_log=${run_dir}/observe_goal.log
sim_pid=''
stack_pid=''

function stop_process_group() {
  local pid=$1
  if [[ -n ${pid} ]] && kill -0 ${pid} 2>/dev/null; then
    /bin/kill -TERM -- -${pid} 2>/dev/null || true
    for attempt in {1..30}; do
      if ! kill -0 ${pid} 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 ${pid} 2>/dev/null; then
      /bin/kill -KILL -- -${pid} 2>/dev/null || true
    fi
    wait ${pid} 2>/dev/null || true
  fi
}

function cleanup() {
  trap - EXIT INT TERM
  stop_process_group ${stack_pid}
  stop_process_group ${sim_pid}
}
trap cleanup EXIT
trap 'exit 130' INT TERM

sim_args=(
  ${conda_bin} run --no-capture-output -n trash_collection
  d1-mujoco-sim --ros-camera --ros-scene
)
if [[ ${headless} == true ]]; then
  sim_args+=(--headless)
fi

print '[1/3] Starting MuJoCo and the calibrated wrist camera...'
cd ${workspace}
setsid ${sim_args[@]} >${sim_log} 2>&1 &
sim_pid=$!

print '[2/3] Starting ros2_control, MoveIt, and ObserveTarget...'
setsid ${conda_bin} run --no-capture-output -n trash_collection \
  ros2 launch d1_manipulation observe_target.launch.py launch_rviz:=${rviz} backend:=simulation \
  >${stack_log} 2>&1 &
stack_pid=$!

ready=false
for attempt in {1..140}; do
  if ! kill -0 ${sim_pid} 2>/dev/null; then
    print -u2 'MuJoCo exited before the observation stack became ready.'
    tail -n 50 ${sim_log} >&2
    exit 1
  fi
  if ! kill -0 ${stack_pid} 2>/dev/null; then
    print -u2 'The ROS 2 observation stack exited before becoming ready.'
    tail -n 80 ${stack_log} >&2
    exit 1
  fi
  if grep -q 'You can start planning now' ${stack_log} && \
     grep -q 'ObserveTarget action server ready' ${stack_log} && \
     grep -q 'DetectTarget ready' ${stack_log} && \
     grep -Eq 'Configured and activated.*arm_controller' ${stack_log} && \
     grep -Eq 'Configured and activated.*gripper_controller' ${stack_log} && \
     grep -Eq 'Configured and activated.*joint_state_broadcaster' ${stack_log} && \
     ros2 topic list 2>/dev/null | grep -qx '/d1_mujoco/object_meshes'; then
    ready=true
    break
  fi
  sleep 0.5
done

if [[ ${ready} != true ]]; then
  print -u2 'Timed out waiting for controllers, MoveIt, and ObserveTarget.'
  tail -n 100 ${stack_log} >&2
  exit 1
fi

if [[ ${server_only} == true ]]; then
  print
  print 'ObserveTarget and DetectTarget are ready for manual calls in another sourced terminal.'
  print
  print 'ros2 action send_goal /arm/debug/observe_target \
  d1_manipulation/action/ObserveTarget \
  "{target: {header: {frame_id: go2_base}, point: {x: 0.45, y: 0.0, z: -0.14}}}" \
  --feedback'
  print
  print "Logs: ${run_dir}"
  print 'Press Ctrl-C here to stop the stack.'
  while kill -0 ${sim_pid} 2>/dev/null && kill -0 ${stack_pid} 2>/dev/null; do
    sleep 1
  done
  print -u2 'A stack process exited unexpectedly.'
  tail -n 60 ${sim_log} ${stack_log} >&2
  exit 1
fi

wait_for_enter \
  'RViz is ready. Inspect the initial state, then press Enter to observe the cube...'

print '[3/3] Sending the seed-0 yellow cube centre to ObserveTarget...'
set +e
ros2 run d1_manipulation observe_seed_cube 2>&1 | tee ${goal_log}
goal_status=${pipestatus[1]}
set -e

if (( goal_status == 0 )) && grep -q 'OBSERVE SUCCEEDED' ${goal_log} && \
   grep -q 'DETECTION SUCCEEDED' ${goal_log}; then
  print
  print 'ACCEPTANCE PASSED: observation executed and the fresh RGB-D target was detected.'
  print "Logs: ${run_dir}"
  wait_for_enter \
    'Observation state is held. Inspect RGB/Depth and markers, then press Enter to close...'
  exit 0
fi

print -u2
print -u2 'ACCEPTANCE FAILED. Diagnostic summary:'
grep -E \
  'TF unavailable|All ordered|execution failed|State tolerances failed|PATH_TOLERANCE|internal error|OBSERVE FAILED|DETECTION FAILED' \
  ${goal_log} ${stack_log} 2>/dev/null | tail -n 20 >&2 || true
print -u2 "Full logs: ${run_dir}"
exit 1
