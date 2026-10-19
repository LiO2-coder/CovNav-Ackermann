#!/usr/bin/env bash
set -Eeuo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-noetic}"
ROS_PREFIX="${ROS_PREFIX:-/opt/ros}"
ROS_SETUP="${ROS_PREFIX}/${ROS_DISTRO_NAME}/setup.bash"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="${WS_ROOT}/devel/setup.bash"
LAUNCH_CMD="roslaunch ackermann_car_gazebo coverage_navigation_gui.launch"
TERMINAL_TITLE="coverage_navigation_gui"

log()
{
  printf '[run] %s\n' "$*"
}

die()
{
  printf '[run:error] %s\n' "$*" >&2
  exit 1
}

kill_gazebo_leftovers()
{
  local gazebo_patterns=(
    gzclient
    gzserver
    gazebo
  )

  log "Killing leftover Gazebo processes"
  for pattern in "${gazebo_patterns[@]}"; do
    pkill -TERM -x "${pattern}" 2>/dev/null || killall -TERM "${pattern}" 2>/dev/null || true
  done

  sleep 1

  for pattern in "${gazebo_patterns[@]}"; do
    pkill -KILL -x "${pattern}" 2>/dev/null || killall -KILL "${pattern}" 2>/dev/null || true
  done
}

terminal_command()
{
  local script_path=$1

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${TERMINAL_TITLE}" -- bash "${script_path}"
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash "${script_path}"
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="${TERMINAL_TITLE}" --command "bash ${script_path}"
  elif command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -p tabtitle="${TERMINAL_TITLE}" -e bash "${script_path}"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "${TERMINAL_TITLE}" -e bash "${script_path}"
  else
    die "No supported terminal emulator found. Run manually: source '${ROS_SETUP}' && source '${WS_SETUP}' && ${LAUNCH_CMD}"
  fi
}

[[ -f "${ROS_SETUP}" ]] || die "ROS ${ROS_DISTRO_NAME} setup not found: ${ROS_SETUP}"
[[ -f "${WS_SETUP}" ]] || die "Workspace setup not found: ${WS_SETUP}. Run ./install.sh first."

kill_gazebo_leftovers

log "Sourcing ROS and workspace setup"
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${WS_SETUP}"

log "Opening new terminal for: ${LAUNCH_CMD}"
launcher_script="$(mktemp "${TMPDIR:-/tmp}/ackermann_run.XXXXXX.sh")"
cat >"${launcher_script}" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
source "${ROS_SETUP}"
source "${WS_SETUP}"
cd "${WS_ROOT}"
${LAUNCH_CMD}
exec bash
EOF
chmod +x "${launcher_script}"
terminal_command "${launcher_script}"
