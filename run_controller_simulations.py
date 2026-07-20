#!/usr/bin/env python3
"""Run paired Fuzzy PID and PID simulations sequentially."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time


WORKSPACE = Path(__file__).resolve().parent
SETUP_FILE = WORKSPACE / "install" / "setup.bash"

LOG_DIR = WORKSPACE / "logs"
CONTROLLERS = (
    (
        "Fuzzy PID",
        "fuzzy_pid",
        LOG_DIR / "fuzzy_pid_velocity_log.csv",
    ),
    (
        "PID",
        "pid",
        LOG_DIR / "pid_velocity_log.csv",
    ),
)


def start_launch(command: str) -> subprocess.Popen[bytes]:
    """Start one ROS launch in its own process group after sourcing the workspace."""
    ros_log_dir = WORKSPACE / "logs" / "ros"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    shell_command = (
        f"source {shlex.quote(str(SETUP_FILE))} && "
        f"export ROS_LOG_DIR={shlex.quote(str(ros_log_dir))} && "
        f"exec {command}"
    )
    return subprocess.Popen(
        ["bash", "-c", shell_command],
        cwd=WORKSPACE,
        start_new_session=True,
    )


def stop_launches(processes: list[subprocess.Popen[bytes]]) -> None:
    """Stop launch files and all of their child processes, in reverse order."""
    running = [process for process in reversed(processes) if process.poll() is None]
    for process in running:
        os.killpg(process.pid, signal.SIGINT)

    deadline = time.monotonic() + 10.0
    for process in running:
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)

    deadline = time.monotonic() + 5.0
    for process in running:
        try:
            process.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def wait_for_log_start(
    log_path: Path,
    process: subprocess.Popen[bytes],
    timeout: float,
) -> None:
    """Wait until Gazebo is ready and the selected controller's logger starts."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(
                f"Simulation launch stopped during startup (exit code {return_code})."
            )
        if log_path.is_file():
            # Wait for the first data row (header + at least 1 row)
            try:
                with log_path.open("r", encoding="utf-8") as log_file:
                    if sum(1 for _ in log_file) >= 2:
                        return
            except OSError:
                pass
        time.sleep(0.25)
    raise RuntimeError(
        f"Gazebo/controller did not become ready within {timeout:g} seconds."
    )


def wait_for_duration(
    log_path: Path,
    duration: float,
    processes: list[subprocess.Popen[bytes]],
    timeout: float,
) -> None:
    """Wait until the simulation log records the target duration."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for process in processes:
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"A launch process stopped unexpectedly (exit code {return_code})."
                )
        try:
            with log_path.open("r", encoding="utf-8") as f:
                lines = f.readlines()
                if len(lines) > 1:
                    last_line = lines[-1]
                    parts = last_line.split(",")
                    if len(parts) > 0:
                        try:
                            current_time = float(parts[0])
                            if current_time >= duration:
                                return
                        except ValueError:
                            pass
        except OSError:
            pass
        time.sleep(0.5)
    raise RuntimeError(
        f"Simulation did not reach {duration:g} sim-seconds within {timeout:g} real-seconds."
    )


def run_simulation(
    name: str,
    controller: str,
    scenario: str,
    seed: int,
    log_path: Path,
    startup_timeout: float,
    duration: float,
    real_timeout: float,
    vehicle_model: str,
    gazebo_gui: str,
) -> None:
    processes: list[subprocess.Popen[bytes]] = []
    print(
        f"\n{'=' * 68}\nStarting {name} | Scenario: {scenario} | "
        f"Seed: {seed}\n{'=' * 68}",
        flush=True,
    )

    try:
        scenario_file = (
            WORKSPACE / "src" / "car_controller" / "config" /
            "simulation_scenarios" / f"{scenario}.yaml"
        )
        if not scenario_file.is_file():
            raise FileNotFoundError(f"Scenario file not found: {scenario_file}")
        if vehicle_model == "gazebo_effort" and scenario not in {
            "baseline", "sensor_noise"
        }:
            raise ValueError(
                "gazebo_effort currently supports baseline and sensor_noise scenarios"
            )

        command = (
            "ros2 launch car_simulation prius_sonoma_control.launch.py "
            f"controller:={controller} "
            f"scenario:={shlex.quote(scenario)} "
            f"scenario_file:={shlex.quote(str(scenario_file))} "
            f"random_seed:={seed} "
            f"vehicle_model:={vehicle_model} "
            f"gazebo_gui:={gazebo_gui}"
        )
        simulation = start_launch(command)
        processes.append(simulation)

        print("Waiting for Gazebo and Prius odometry...", flush=True)
        wait_for_log_start(log_path, simulation, startup_timeout)

        print(f"Running {name} until {duration:g} sim-seconds...", flush=True)
        wait_for_duration(log_path, duration, processes, real_timeout)

    finally:
        print(f"Stopping {name} simulation...", flush=True)
        stop_launches(processes)


def indexed_path(path: Path, vehicle_model: str, scenario: str, seed: int) -> Path:
    return path.with_name(
        f"{path.stem}_{vehicle_model}_{scenario}_{seed}{path.suffix}"
    )


def archive_log(source: Path, vehicle_model: str, scenario: str, seed: int) -> Path:
    if not source.is_file():
        raise RuntimeError(f"Simulation did not create the expected log: {source}")
    destination = indexed_path(source, vehicle_model, scenario, seed)
    shutil.move(str(source), str(destination))
    return destination


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run paired fuzzy PID and PID simulations."
    )
    parser.add_argument(
        "--vehicle-model",
        choices=("longitudinal_sim", "gazebo_effort"),
        default="gazebo_effort",
        help="Vehicle backend (default: gazebo_effort)",
    )
    parser.add_argument(
        "--gazebo-gui",
        choices=("true", "false"),
        default="true",
        help="Whether to open the Gazebo GUI (default: true)",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="baseline",
        help=(
            "Scenario to run (e.g. baseline, grade_step, sensor_noise, combined). "
            "Default: baseline"
        ),
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of paired PID/Fuzzy PID runs (default: 1)",
    )
    parser.add_argument(
        "--base-seed",
        type=int,
        default=1001,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=46.0,
        help="Target simulation time in seconds (default: 46.0)",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=60.0,
        help="Maximum real seconds to wait for startup (default: 60.0)",
    )
    parser.add_argument(
        "--real-timeout",
        type=float,
        default=120.0,
        help=(
            "Maximum real seconds to wait for the simulation duration to complete "
            "(default: 120.0)"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not SETUP_FILE.is_file():
        print(
            f"Missing {SETUP_FILE}. Run 'colcon build' before this script.",
            file=sys.stderr,
        )
        return 1
    if args.runs <= 0:
        print("--runs must be a positive integer.", file=sys.stderr)
        return 2
    if args.base_seed < 0:
        print("--base-seed must be non-negative.", file=sys.stderr)
        return 2
    if args.duration <= 0.0 or args.startup_timeout <= 0.0 or args.real_timeout <= 0.0:
        print(
            "--duration, --startup-timeout, and --real-timeout must be positive.",
            file=sys.stderr,
        )
        return 2

    seeds = [args.base_seed + run_index for run_index in range(args.runs)]
    archived_logs: list[Path] = []
    try:
        for idx, seed in enumerate(seeds, 1):
            print(
                f"\n######## Run {idx}/{args.runs} "
                f"(internal seed: {seed}) ########",
                flush=True,
            )
            for name, controller, log_path in CONTROLLERS:
                log_path.unlink(missing_ok=True)
                run_simulation(
                    name,
                    controller,
                    args.scenario,
                    seed,
                    log_path,
                    args.startup_timeout,
                    args.duration,
                    args.real_timeout,
                    args.vehicle_model,
                    args.gazebo_gui,
                )
                archived_logs.append(
                    archive_log(log_path, args.vehicle_model, args.scenario, seed)
                )
    except KeyboardInterrupt:
        print("\nCancelled by user.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(f"\nSimulation failed: {error}", file=sys.stderr)
        return 1

    print(
        f"\nCompleted {args.runs} simulation pair(s) successfully for "
        f"scenario '{args.scenario}'.",
        flush=True,
    )
    for path in archived_logs:
        print(path)

    print(
        "\nCreate plots with: "
        "python3 logs/plot_velocity_log.py "
        f"--vehicle-model {args.vehicle_model} "
        f"--scenario {args.scenario} --runs {args.runs}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
