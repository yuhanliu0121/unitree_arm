#!/usr/bin/env zsh

set -eo pipefail

workspace=${0:A:h:h:h:h}
conda_bin=/home/tony/miniconda3/bin/conda
ros_setup=/opt/ros/humble/setup.zsh
headless=false

function usage() {
  print 'Usage: ./view_mujoco_camera.zsh [--headless]'
  print '  --headless  Do not open the main MuJoCo viewer'
}

while (( $# > 0 )); do
  case "$1" in
    --headless)
      headless=true
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

source ${ros_setup}
if [[ -r ${workspace}/install/setup.zsh ]]; then
  source ${workspace}/install/setup.zsh
fi
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export PYTHONNOUSERSITE=1
export PYTHONPATH="${workspace}/simulation/d1_mujoco_sim/src${PYTHONPATH:+:${PYTHONPATH}}"

run_dir=$(mktemp -d /tmp/d1_camera_view.XXXXXX)
sim_log=${run_dir}/mujoco_camera.log
sim_pid=''; visualizer_pid=''

function stop_simulator() {
  if [[ -n ${visualizer_pid} ]] && kill -0 ${visualizer_pid} 2>/dev/null; then
    /bin/kill -TERM ${visualizer_pid} 2>/dev/null || true
    wait ${visualizer_pid} 2>/dev/null || true
  fi
  if [[ -n ${sim_pid} ]] && kill -0 ${sim_pid} 2>/dev/null; then
    /bin/kill -TERM -- -${sim_pid} 2>/dev/null || true
    wait ${sim_pid} 2>/dev/null || true
  fi
}
trap stop_simulator EXIT INT TERM

sim_args=(
  ${conda_bin} run --no-capture-output -n trash_collection
  python -m d1_mujoco_sim.cli --ros-camera
)
if [[ ${headless} == true ]]; then
  sim_args+=(--headless)
fi

print 'Starting MuJoCo and the ROS 2 camera bridge...'
cd ${workspace}
if [[ ${headless} == true ]]; then
  setsid env MUJOCO_GL=egl ${sim_args[@]} >${sim_log} 2>&1 &
else
  setsid ${sim_args[@]} >${sim_log} 2>&1 &
fi
sim_pid=$!

ready=false
for attempt in {1..120}; do
  if ! kill -0 ${sim_pid} 2>/dev/null; then
    print -u2 'MuJoCo camera bridge exited during startup.'
    tail -n 60 ${sim_log} >&2
    exit 1
  fi
  if grep -q 'ROS camera streams ready' ${sim_log}; then
    ready=true
    break
  fi
  sleep 0.5
done
if [[ ${ready} != true ]]; then
  print -u2 'Timed out waiting for the camera bridge.'
  tail -n 60 ${sim_log} >&2
  exit 1
fi

print 'Camera bridge ready. Starting RViz...'
print "Logs: ${run_dir}"
ros2 run d1_camera_visualization depth_debug_visualizer >${run_dir}/depth_visualizer.log 2>&1 &
visualizer_pid=$!
rviz2 -d ${workspace}/simulation/d1_mujoco_sim/config/camera.rviz
