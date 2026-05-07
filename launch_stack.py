#!/usr/bin/env python3
"""
Full robot stack launcher — runs directly on the Jetson (no Docker).

Usage:
  python3 launch_stack.py              # launch everything
  python3 launch_stack.py --no-base    # skip diffdrive / ros2_control
  python3 launch_stack.py --no-sensors # skip ZED cameras + perception
  python3 launch_stack.py --no-nav     # skip EKF localization + nav2
  python3 launch_stack.py --no-teleop  # skip joystick teleop
  python3 launch_stack.py --no-lane    # skip YOLOP lane detection
  python3 launch_stack.py --mission    # also launch igvc waypoint mission
  python3 launch_stack.py --rviz       # also launch RViz2

Press Ctrl+C to stop everything.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
WORKSPACE = os.path.dirname(os.path.abspath(__file__))
ROS_SETUP  = '/opt/ros/humble/setup.bash'
WS_SETUP   = os.path.join(WORKSPACE, 'install', 'setup.bash')

# ---------------------------------------------------------------------------
# ANSI colours for per-process labels
# ---------------------------------------------------------------------------
COLOURS = [
    '\033[94m',  # blue
    '\033[92m',  # green
    '\033[93m',  # yellow
    '\033[95m',  # magenta
    '\033[96m',  # cyan
    '\033[91m',  # red
    '\033[97m',  # white
    '\033[33m',  # orange-ish
]
RESET = '\033[0m'
BOLD  = '\033[1m'

# ---------------------------------------------------------------------------
# Build the sourced ROS environment once
# ---------------------------------------------------------------------------
def get_ros_env() -> dict:
    sources = []
    if os.path.exists(ROS_SETUP):
        sources.append(f'source {ROS_SETUP}')
    if os.path.exists(WS_SETUP):
        sources.append(f'source {WS_SETUP}')
    if not sources:
        print(f'{BOLD}ERROR:{RESET} Neither {ROS_SETUP} nor {WS_SETUP} found.')
        print('       Build the workspace first:  colcon build --symlink-install')
        sys.exit(1)
    cmd = ' && '.join(sources) + ' && env'
    raw = subprocess.check_output(['bash', '-c', cmd], cwd=WORKSPACE).decode()
    env = {}
    for line in raw.splitlines():
        if '=' in line:
            k, v = line.split('=', 1)
            env[k] = v

    # If a venv exists, prepend its bin/ to PATH so all subprocesses use its
    # Python and pip-installed packages (numpy<2, torch, ultralytics, etc.)
    venv_bin = os.path.join(WORKSPACE, 'venv', 'bin')
    if os.path.isdir(venv_bin):
        env['PATH'] = venv_bin + os.pathsep + env.get('PATH', '')
        env['VIRTUAL_ENV'] = os.path.join(WORKSPACE, 'venv')
        env.pop('PYTHONHOME', None)
        print(f'{BOLD}venv:{RESET} {venv_bin}')
    else:
        print(f'{BOLD}WARNING:{RESET} No venv found. Run: bash setup_venv.sh')

    return env

# ---------------------------------------------------------------------------
# Process management
# ---------------------------------------------------------------------------
_procs: list[tuple[str, subprocess.Popen]] = []
_lock = threading.Lock()


def _stream(proc: subprocess.Popen, label: str, colour: str) -> None:
    prefix = f'{colour}{BOLD}[{label}]{RESET} '
    for line in proc.stdout:
        sys.stdout.write(prefix + line)
        sys.stdout.flush()


def launch(label: str, cmd: list[str], env: dict, delay: float = 0.0, colour: str = '') -> None:
    if delay:
        time.sleep(delay)
    print(f'{BOLD}>> Starting:{RESET} {label}  ({" ".join(cmd)})')
    proc = subprocess.Popen(
        cmd,
        env=env,
        cwd=WORKSPACE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    with _lock:
        _procs.append((label, proc))
    t = threading.Thread(target=_stream, args=(proc, label, colour), daemon=True)
    t.start()


def shutdown(signum=None, frame=None) -> None:
    print(f'\n{BOLD}Shutting down all processes...{RESET}')
    with _lock:
        procs = list(_procs)
    for label, proc in reversed(procs):
        if proc.poll() is None:
            print(f'  stopping {label}')
            proc.send_signal(signal.SIGINT)
    time.sleep(2)
    for _, proc in procs:
        if proc.poll() is None:
            proc.kill()
    sys.exit(0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description='Launch the full robot stack.')
    parser.add_argument('--no-base',    action='store_true', help='Skip diffdrive / ros2_control')
    parser.add_argument('--no-sensors', action='store_true', help='Skip ZED cameras + perception')
    parser.add_argument('--no-nav',     action='store_true', help='Skip EKF localization + nav2')
    parser.add_argument('--no-teleop',  action='store_true', help='Skip joystick teleop')
    parser.add_argument('--no-lane',    action='store_true', help='Skip YOLOP lane detection')
    parser.add_argument('--mission',    action='store_true', help='Launch IGVC waypoint mission')
    parser.add_argument('--rviz',       action='store_true', help='Launch RViz2')
    args = parser.parse_args()

    if not os.path.exists(WS_SETUP):
        print(f'{BOLD}WARNING:{RESET} {WS_SETUP} not found.')
        print('         Run:  cd {WORKSPACE} && colcon build --symlink-install')
        print('         Continuing with /opt/ros/humble only...\n')

    env = get_ros_env()
    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    colour_idx = 0

    def next_colour() -> str:
        nonlocal colour_idx
        c = COLOURS[colour_idx % len(COLOURS)]
        colour_idx += 1
        return c

    print(f'\n{BOLD}=== Robot Stack Launcher ==={RESET}')
    print(f'Workspace : {WORKSPACE}')
    print(f'ROS env   : {WS_SETUP if os.path.exists(WS_SETUP) else ROS_SETUP}')
    print()

    threads = []

    # ── 1. Robot base (diffdrive + ros2_control + RSP) ──────────────────────
    if not args.no_base:
        t = threading.Thread(target=launch, kwargs=dict(
            label='base',
            cmd=['ros2', 'launch', 'my_robot_description', 'diffdrive_physical.launch.py'],
            env=env,
            delay=0.0,
            colour=next_colour(),
        ))
        t.start(); threads.append(t)

    # ── 2. Sensors + perception (ZED cameras, LiDAR, YOLO) ──────────────────
    if not args.no_sensors:
        t = threading.Thread(target=launch, kwargs=dict(
            label='sensors',
            cmd=['ros2', 'launch', 'my_robot_bringup', 'sensor_fusion.launch.py'],
            env=env,
            delay=3.0,
            colour=next_colour(),
        ))
        t.start(); threads.append(t)

    # ── 3. EKF localization + GPS ────────────────────────────────────────────
    if not args.no_nav:
        t = threading.Thread(target=launch, kwargs=dict(
            label='localization',
            cmd=['ros2', 'launch', 'my_robot_localization', 'gps_localization_launch.py'],
            env=env,
            delay=5.0,
            colour=next_colour(),
        ))
        t.start(); threads.append(t)

    # ── 4. Nav2 ──────────────────────────────────────────────────────────────
    if not args.no_nav:
        t = threading.Thread(target=launch, kwargs=dict(
            label='nav2',
            cmd=['ros2', 'launch', 'my_robot_navigation', 'nav2.launch.py'],
            env=env,
            delay=8.0,
            colour=next_colour(),
        ))
        t.start(); threads.append(t)

    # ── 5. YOLOP lane detection ──────────────────────────────────────────────
    if not args.no_lane:
        t = threading.Thread(target=launch, kwargs=dict(
            label='lane',
            cmd=['ros2', 'run', 'yolop_lane_ros2', 'yolop_lane_node'],
            env=env,
            delay=5.0,
            colour=next_colour(),
        ))
        t.start(); threads.append(t)

    # ── 6. Camera-to-points ──────────────────────────────────────────────────
    t = threading.Thread(target=launch, kwargs=dict(
        label='cam2pts',
        cmd=['ros2', 'run', 'my_robot_cam2points', 'camCoords'],
        env=env,
        delay=5.0,
        colour=next_colour(),
    ))
    t.start(); threads.append(t)

    # ── 7. Teleop (joystick) ─────────────────────────────────────────────────
    if not args.no_teleop:
        t = threading.Thread(target=launch, kwargs=dict(
            label='teleop',
            cmd=['ros2', 'launch', 'my_robot_teleop', 'teleop_launch.py'],
            env=env,
            delay=5.0,
            colour=next_colour(),
        ))
        #t.start(); threads.append(t)

    # ── 8. IGVC mission (opt-in) ─────────────────────────────────────────────
    if args.mission:
        t = threading.Thread(target=launch, kwargs=dict(
            label='mission',
            cmd=['ros2', 'launch', 'my_robot_igvc_mission', 'mission_launch.py'],
            env=env,
            delay=10.0,
            colour=next_colour(),
        ))
        #t.start(); threads.append(t)

    # ── 9. RViz2 (opt-in) ────────────────────────────────────────────────────
    if args.rviz:
        rviz_cfg = os.path.join(WORKSPACE, 'src', 'my_robot_bringup', 'launch', 'zed_rviz.rviz')
        cmd = ['ros2', 'run', 'rviz2', 'rviz2']
        if os.path.exists(rviz_cfg):
            cmd += ['-d', rviz_cfg]
        t = threading.Thread(target=launch, kwargs=dict(
            label='rviz',
            cmd=cmd,
            env=env,
            delay=6.0,
            colour=next_colour(),
        ))
        #t.start(); threads.append(t)

    for t in threads:
        t.join()

    # Keep main thread alive (all output comes from daemon reader threads)
    print(f'\n{BOLD}All processes launched. Press Ctrl+C to stop.{RESET}\n')
    warned = set()
    while True:
        with _lock:
            for label, proc in _procs:
                if label not in warned and proc.poll() is not None:
                    print(f'{BOLD}WARNING:{RESET} [{label}] exited with code {proc.returncode}')
                    warned.add(label)
        time.sleep(5)


if __name__ == '__main__':
    main()
