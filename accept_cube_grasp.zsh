#!/usr/bin/env zsh

set -eo pipefail

workspace=${0:A:h}
conda_bin=/home/tony/miniconda3/bin/conda
ros_setup=/opt/ros/humble/setup.zsh
headless=false
rviz=false

function usage() {
  print 'Usage: ./accept_cube_grasp.zsh [--headless] [--rviz]'
  print '  --headless  Do not open the MuJoCo viewer'
  print '  --rviz      Open RViz and pause before/after grasp for inspection'
}

function wait_for_enter() {
  local prompt=$1
  if [[ ${rviz} == true && -t 0 ]]; then
    print
    print -n "${prompt}"
    local inspection_reply
    IFS= read -r inspection_reply
  fi
}

while (( $# > 0 )); do
  case "$1" in
    --headless)
      headless=true
      ;;
    --rviz)
      rviz=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
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

run_dir=$(mktemp -d /tmp/d1_cube_accept.XXXXXX)
sim_log=${run_dir}/mujoco.log
stack_log=${run_dir}/moveit.log
pick_log=${run_dir}/pick.log
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
  d1-mujoco-sim
)
if [[ ${headless} == true ]]; then
  sim_args+=(--headless)
fi
if [[ ${rviz} == true ]]; then
  sim_args+=(--ros-camera --ros-scene)
fi

print '[1/3] Starting MuJoCo simulation...'
cd ${workspace}
setsid ${sim_args[@]} >${sim_log} 2>&1 &
sim_pid=$!

print '[2/3] Starting ros2_control and MoveIt...'
setsid ros2 launch d1_moveit_config move_group.launch.py launch_rviz:=${rviz} backend:=simulation \
  >${stack_log} 2>&1 &
stack_pid=$!

ready=false
for attempt in {1..120}; do
  if ! kill -0 ${sim_pid} 2>/dev/null; then
    print -u2 'MuJoCo exited before the control stack became ready.'
    tail -n 40 ${sim_log} >&2
    exit 1
  fi
  if ! kill -0 ${stack_pid} 2>/dev/null; then
    print -u2 'The ROS 2/MoveIt stack exited before becoming ready.'
    tail -n 60 ${stack_log} >&2
    exit 1
  fi

  # The launch output is a bounded readiness source and avoids ROS CLI service
  # discovery calls blocking indefinitely while DDS is still converging.
  if grep -q 'You can start planning now' ${stack_log} && \
     grep -Eq 'Configured and activated.*arm_controller' ${stack_log} && \
     grep -Eq 'Configured and activated.*gripper_controller' ${stack_log} && \
     grep -Eq 'Configured and activated.*joint_state_broadcaster' ${stack_log}; then
    ready=true
    break
  fi
  sleep 0.5
done

if [[ ${ready} != true ]]; then
  print -u2 'Timed out waiting 60 seconds for controllers and MoveIt.'
  tail -n 80 ${stack_log} >&2
  exit 1
fi

wait_for_enter \
  'RViz is ready. Inspect/toggle the layers, then press Enter to start grasp...'

print '[3/3] Controllers are active; running the fixed-cube grasp...'
set +e
ros2 launch d1_moveit_config pick_cube.launch.py 2>&1 \
  | tee ${pick_log} \
  | grep --line-buffered -E 'd1_pick_cube|GRASP (SUCCEEDED|FAILED)|process has died'
launch_status=${pipestatus[1]}
set -e

if (( launch_status == 0 )) && grep -q 'GRASP SUCCEEDED' ${pick_log}; then
  print
  print 'ACCEPTANCE PASSED: the physical MuJoCo cube remained lifted.'
  print "Logs: ${run_dir}"
  wait_for_enter \
    'Final grasp state is held. Press Enter to close RViz and MuJoCo...'
  exit 0
fi

print -u2
print -u2 'ACCEPTANCE FAILED. Diagnostic summary:'
grep -E \
  'State tolerances failed|Position Error:|PATH_TOLERANCE|feedback stale|GRASP FAILED|execution failed' \
  ${pick_log} ${stack_log} 2>/dev/null | tail -n 12 >&2 || true
print -u2 "Full logs: ${run_dir}"
exit 1
