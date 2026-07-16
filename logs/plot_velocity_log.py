#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_FUZZY_LOG = SCRIPT_DIR / "fuzzy_pid_velocity_log.csv"
DEFAULT_PID_LOG = SCRIPT_DIR / "pid_velocity_log.csv"
DEFAULT_OUTPUT = SCRIPT_DIR / "velocity_tracking_comparison.png"
DEFAULT_GAINS_OUTPUT = SCRIPT_DIR / "fuzzy_pid_adaptive_gains.png"


def read_velocity_log(path):
    time_sec = []
    v_ref = []
    v_actual = []
    v_measured = []
    error = []
    target_acceleration = []
    applied_acceleration = []
    adaptive_kp = []
    adaptive_ki = []
    adaptive_kd = []
    fuzzy_error_e = []
    fuzzy_error_change_ec = []
    fuzzy_normalized_error_e = []
    fuzzy_normalized_error_change_ec = []
    p_term = []
    i_term = []
    d_term = []
    grade_percent = []

    with path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        # Check required columns for Schema v2
        required_columns = {
            "time_sec",
            "target_velocity_mps",
            "true_velocity_mps",
            "true_velocity_error_mps",
            "target_acceleration_mps2",
            "applied_acceleration_mps2",
        }
        missing_columns = required_columns.difference(reader.fieldnames or [])
        if missing_columns:
            missing = ", ".join(sorted(missing_columns))
            raise RuntimeError(f"Missing columns in {path}: {missing}")

        gain_columns = {"adaptive_kp", "adaptive_ki", "adaptive_kd"}
        has_adaptive_gains = gain_columns.issubset(reader.fieldnames or [])
        pid_terms_columns = {"p_term_mps2", "i_term_mps2", "d_term_mps2"}
        has_pid_terms = pid_terms_columns.issubset(reader.fieldnames or [])
        has_measured_vel = "measured_velocity_mps" in (reader.fieldnames or [])
        has_grade = "grade_percent" in (reader.fieldnames or [])
        has_fuzzy_inputs = {
            "fuzzy_error_e_mps",
            "fuzzy_error_change_ec_mps2",
        }.issubset(reader.fieldnames or [])
        has_normalized_fuzzy_inputs = {
            "fuzzy_normalized_error_e",
            "fuzzy_normalized_error_change_ec",
        }.issubset(reader.fieldnames or [])
        has_legacy_fuzzy_inputs = {
            "controller_error_mps",
            "controller_error_derivative_mps2",
        }.issubset(reader.fieldnames or [])
        has_controller_error = "controller_velocity_error_mps" in (reader.fieldnames or [])

        for row in reader:
            time_sec.append(float(row["time_sec"]))
            v_ref.append(float(row["target_velocity_mps"]))
            v_actual.append(float(row["true_velocity_mps"]))
            error.append(float(row["true_velocity_error_mps"]))
            target_acceleration.append(float(row["target_acceleration_mps2"]))
            applied_acceleration.append(float(row["applied_acceleration_mps2"]))

            if has_measured_vel:
                v_measured.append(float(row["measured_velocity_mps"]))
            else:
                v_measured.append(float(row["true_velocity_mps"]))

            if has_grade:
                grade_percent.append(float(row["grade_percent"]))

            if has_adaptive_gains:
                adaptive_kp.append(float(row["adaptive_kp"]))
                adaptive_ki.append(float(row["adaptive_ki"]))
                adaptive_kd.append(float(row["adaptive_kd"]))

            if has_fuzzy_inputs:
                fuzzy_error_e.append(float(row["fuzzy_error_e_mps"]))
                fuzzy_error_change_ec.append(float(row["fuzzy_error_change_ec_mps2"]))
            elif has_legacy_fuzzy_inputs:
                fuzzy_error_e.append(float(row["controller_error_mps"]))
                fuzzy_error_change_ec.append(float(row["controller_error_derivative_mps2"]))
            elif has_controller_error:
                fuzzy_error_e.append(float(row["controller_velocity_error_mps"]))

            if has_normalized_fuzzy_inputs:
                fuzzy_normalized_error_e.append(float(row["fuzzy_normalized_error_e"]))
                fuzzy_normalized_error_change_ec.append(
                    float(row["fuzzy_normalized_error_change_ec"])
                )

            if has_pid_terms:
                p_term.append(float(row["p_term_mps2"]))
                i_term.append(float(row["i_term_mps2"]))
                d_term.append(float(row["d_term_mps2"]))

    if not time_sec:
        raise RuntimeError(f"No samples found in {path}")

    if fuzzy_error_e and not fuzzy_error_change_ec:
        fuzzy_error_change_ec = estimate_derivative(time_sec, fuzzy_error_e)

    return {
        "time_sec": time_sec,
        "v_ref": v_ref,
        "v_actual": v_actual,
        "v_measured": v_measured,
        "error": error,
        "target_acceleration": target_acceleration,
        "applied_acceleration": applied_acceleration,
        "adaptive_kp": adaptive_kp,
        "adaptive_ki": adaptive_ki,
        "adaptive_kd": adaptive_kd,
        "fuzzy_error_e": fuzzy_error_e,
        "fuzzy_error_change_ec": fuzzy_error_change_ec,
        "fuzzy_normalized_error_e": fuzzy_normalized_error_e,
        "fuzzy_normalized_error_change_ec": fuzzy_normalized_error_change_ec,
        "p_term": p_term,
        "i_term": i_term,
        "d_term": d_term,
        "grade_percent": grade_percent,
    }


def estimate_derivative(time_sec, values):
    derivative = []
    previous_time = None
    previous_value = None
    for time, value in zip(time_sec, values):
        if previous_time is None:
            derivative.append(0.0)
        else:
            dt = time - previous_time
            derivative.append((value - previous_value) / dt if dt > 0.0 else 0.0)
        previous_time = time
        previous_value = value
    return derivative


def normalize_time(data):
    start_time = data["time_sec"][0]
    # Infer the profile origin from the first rising reference segment. This
    # also handles logs whose logger joined after v_ref had already exceeded 1 m/s.
    if data["v_ref"]:
        for index in range(1, len(data["time_sec"])):
            dt = data["time_sec"][index] - data["time_sec"][index - 1]
            dv = data["v_ref"][index] - data["v_ref"][index - 1]
            if dt > 0.0 and dv > 1e-6:
                slope = dv / dt
                start_time = (
                    data["time_sec"][index - 1]
                    - data["v_ref"][index - 1] / slope
                )
                break
    normalized = dict(data)
    normalized["time_sec"] = [time - start_time for time in data["time_sec"]]
    return normalized


def set_symmetric_ylim(axis, *series, min_half_range=0.08, padding=1.15):
    max_abs = 0.0
    for values in series:
        if values:
            max_abs = max(max_abs, max(abs(value) for value in values))
    half_range = max(min_half_range, max_abs * padding)
    axis.set_ylim(-half_range, half_range)


def plot_velocity_logs(
    fuzzy_path, pid_path, output_path, gains_output_path, show_plot, plt=None, scenario="unknown"
):
    if plt is None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError(
                "matplotlib is required. Install it with: sudo apt install python3-matplotlib"
            ) from exc

    fuzzy = normalize_time(read_velocity_log(fuzzy_path))
    pid = normalize_time(read_velocity_log(pid_path))

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))

    axes[0].plot(fuzzy["time_sec"], fuzzy["v_ref"], label="v_ref", linewidth=2.2)
    axes[0].plot(
        fuzzy["time_sec"],
        fuzzy["v_actual"],
        label="v_actual_fuzzy_pid",
        linewidth=2.0,
    )
    axes[0].plot(
        pid["time_sec"],
        pid["v_actual"],
        label="v_actual_pid",
        linewidth=2.0,
    )
    # If sensor noise is present, plot measured velocity faintly
    has_sensor_noise = any(
        abs(actual - measured) > 1e-3
        for actual, measured in zip(fuzzy["v_actual"], fuzzy["v_measured"])
    )
    if has_sensor_noise:
        axes[0].plot(
            fuzzy["time_sec"],
            fuzzy["v_measured"],
            label="v_measured_fuzzy_pid",
            alpha=0.3,
            linewidth=1.0,
        )
    axes[0].set_ylabel("Velocity (m/s)")
    axes[0].set_xlabel("Time (s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(
        fuzzy["time_sec"],
        fuzzy["target_acceleration"],
        label="target_accel_fuzzy_pid",
        color="tab:green",
        linewidth=2.0,
    )
    axes[1].plot(
        pid["time_sec"],
        pid["target_acceleration"],
        label="target_accel_pid",
        color="tab:olive",
        linewidth=2.0,
        linestyle="--",
    )
    axes[1].plot(
        fuzzy["time_sec"],
        fuzzy["applied_acceleration"],
        label="applied_accel_fuzzy_pid (plant)",
        color="black",
        linewidth=1.5,
        alpha=0.6,
        linestyle=":",
    )
    axes[1].set_ylabel("Acceleration (m/s²)")
    axes[1].set_xlabel("Time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].plot(
        fuzzy["time_sec"],
        fuzzy["error"],
        label="error_fuzzy_pid",
        color="tab:red",
        linewidth=2.0,
    )
    axes[2].plot(
        pid["time_sec"],
        pid["error"],
        label="error_pid",
        color="tab:purple",
        linewidth=2.0,
        linestyle="--",
    )
    axes[2].axhline(0.0, color="black", linewidth=1.0, alpha=0.5)
    set_symmetric_ylim(axes[2], fuzzy["error"], pid["error"])

    # If there is grade variation, plot it on secondary axis
    if any(abs(g) > 1e-3 for g in fuzzy["grade_percent"]):
        ax2_grade = axes[2].twinx()
        ax2_grade.plot(
            fuzzy["time_sec"],
            fuzzy["grade_percent"],
            color="black",
            alpha=0.2,
            linestyle="-.",
        )
        ax2_grade.set_ylabel("Grade (%)", color="black", alpha=0.5)
        ax2_grade.tick_params(axis="y", labelcolor="gray")

    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Error (m/s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper left")
    for axis in axes:
        axis.tick_params(axis="x", labelbottom=True)

    fig.suptitle(f"Velocity Tracking Comparison (Scenario: {scenario})")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"Saved plot to {output_path}")

    # Plot gains if present
    if fuzzy["adaptive_kp"]:
        has_pid_terms = bool(fuzzy["p_term"])
        has_fuzzy_inputs = bool(fuzzy["fuzzy_error_e"]) and bool(fuzzy["fuzzy_error_change_ec"])
        subplot_count = 1 + int(has_fuzzy_inputs) + int(has_pid_terms)
        gains_fig, gains_axes = plt.subplots(
            subplot_count, 1, sharex=True, figsize=(12, 3.8 * subplot_count)
        )
        if subplot_count == 1:
            gains_axes = [gains_axes]

        gain_series = (
            ("adaptive_kp", "Kp", "tab:blue"),
            ("adaptive_ki", "Ki", "tab:orange"),
            ("adaptive_kd", "Kd", "tab:green"),
        )
        for key, label, color in gain_series:
            gains_axes[0].plot(
                fuzzy["time_sec"],
                fuzzy[key],
                label=label,
                color=color,
                linewidth=1.8,
            )
        gains_axes[0].set_ylabel("Adaptive gain")
        gains_axes[0].set_xlabel("Time (s)")
        gains_axes[0].grid(True, alpha=0.3)
        gains_axes[0].legend(loc="best")

        next_axis = 1
        if has_fuzzy_inputs:
            fuzzy_input_axis = gains_axes[next_axis]
            fuzzy_input_axis.plot(
                fuzzy["time_sec"],
                fuzzy["fuzzy_error_e"],
                label="e = velocity error",
                color="tab:red",
                linewidth=1.7,
            )
            fuzzy_input_axis.axhline(0.0, color="black", linewidth=1.0, alpha=0.5)
            fuzzy_input_axis.set_ylabel("e (m/s)", color="tab:red")
            fuzzy_input_axis.set_xlabel("Time (s)")
            fuzzy_input_axis.grid(True, alpha=0.3)
            fuzzy_input_axis.tick_params(axis="y", labelcolor="tab:red")

            fuzzy_ec_axis = fuzzy_input_axis.twinx()
            fuzzy_ec_axis.plot(
                fuzzy["time_sec"],
                fuzzy["fuzzy_error_change_ec"],
                label="ec = error derivative",
                color="tab:purple",
                linewidth=1.2,
                alpha=0.85,
            )
            fuzzy_ec_axis.set_ylabel("ec (m/s²)", color="tab:purple")
            fuzzy_ec_axis.tick_params(axis="y", labelcolor="tab:purple")

            if fuzzy["fuzzy_normalized_error_e"] and fuzzy["fuzzy_normalized_error_change_ec"]:
                fuzzy_input_axis.plot(
                    fuzzy["time_sec"],
                    fuzzy["fuzzy_normalized_error_e"],
                    label="scaled e",
                    color="tab:pink",
                    linewidth=1.0,
                    alpha=0.45,
                    linestyle="--",
                )
                fuzzy_ec_axis.plot(
                    fuzzy["time_sec"],
                    fuzzy["fuzzy_normalized_error_change_ec"],
                    label="scaled ec",
                    color="tab:brown",
                    linewidth=1.0,
                    alpha=0.45,
                    linestyle=":",
                )

            lines, labels = fuzzy_input_axis.get_legend_handles_labels()
            ec_lines, ec_labels = fuzzy_ec_axis.get_legend_handles_labels()
            fuzzy_input_axis.legend(lines + ec_lines, labels + ec_labels, loc="best")

            next_axis += 1

        if has_pid_terms:
            control_axis = gains_axes[next_axis]
            control_series = (
                ("p_term", "P_term", "tab:blue", 1.5),
                ("i_term", "I_term", "tab:orange", 1.5),
                ("d_term", "D_term", "tab:purple", 1.5),
            )
            for key, label, color, linewidth in control_series:
                control_axis.plot(
                    fuzzy["time_sec"],
                    fuzzy[key],
                    label=label,
                    color=color,
                    linewidth=linewidth,
                )
            control_axis.plot(
                fuzzy["time_sec"],
                fuzzy["target_acceleration"],
                label="target_accel",
                color="tab:green",
                linewidth=2.0,
                alpha=0.7,
            )
            control_axis.axhline(0.0, color="black", linewidth=1.0, alpha=0.5)
            control_axis.set_ylabel("Acceleration Terms (m/s²)")
            control_axis.set_xlabel("Time (s)")
            control_axis.grid(True, alpha=0.3)
            control_axis.legend(loc="best")

        gains_axes[-1].set_xlabel("Time (s)")
        for axis in gains_axes:
            axis.tick_params(axis="x", labelbottom=True)
        gains_fig.suptitle(f"Fuzzy PID Adaptive Gains and Terms (Scenario: {scenario})")
        gains_fig.tight_layout()
        gains_fig.savefig(gains_output_path, dpi=150, bbox_inches="tight")
        print(f"Saved adaptive gains plot to {gains_output_path}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)
        if fuzzy["adaptive_kp"]:
            plt.close(gains_fig)


def indexed_path(path, vehicle_model, scenario, seed):
    return path.with_name(
        f"{path.stem}_{vehicle_model}_{scenario}_{seed}{path.suffix}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Plot fuzzy PID and PID velocity tracking CSV logs."
    )
    parser.add_argument(
        "--vehicle-model",
        choices=("longitudinal_sim", "gazebo_effort"),
        default="gazebo_effort",
        help="Vehicle backend used to name the logs (default: gazebo_effort)",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="baseline",
        help="Scenario to plot (e.g. baseline, grade_step). Default: baseline",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of paired simulation runs to plot (default: 1)",
    )
    parser.add_argument(
        "--base-seed",
        type=int,
        default=1001,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--fuzzy",
        default=str(DEFAULT_FUZZY_LOG),
        help=f"Fuzzy PID CSV base log path. Default: {DEFAULT_FUZZY_LOG}",
    )
    parser.add_argument(
        "--gains-output",
        default=str(DEFAULT_GAINS_OUTPUT),
        help=f"Adaptive gains output base image path. Default: {DEFAULT_GAINS_OUTPUT}",
    )
    parser.add_argument(
        "--pid",
        default=str(DEFAULT_PID_LOG),
        help=f"PID CSV base log path. Default: {DEFAULT_PID_LOG}",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help=f"Output base image path. Default: {DEFAULT_OUTPUT}",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show the plot window after saving.",
    )
    args = parser.parse_args()

    if args.runs <= 0:
        parser.error("--runs must be a positive integer")
    if args.base_seed < 0:
        parser.error("--base-seed must be non-negative")

    fuzzy_path = Path(args.fuzzy).expanduser()
    pid_path = Path(args.pid).expanduser()
    output_path = Path(args.output).expanduser()
    gains_output_path = Path(args.gains_output).expanduser()

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required. Install it with: sudo apt install python3-matplotlib"
        ) from exc

    plotted_count = 0
    seeds = [args.base_seed + run_index for run_index in range(args.runs)]
    for seed in seeds:
        indexed_fuzzy = indexed_path(
            fuzzy_path, args.vehicle_model, args.scenario, seed
        )
        indexed_pid = indexed_path(pid_path, args.vehicle_model, args.scenario, seed)
        indexed_output = indexed_path(
            output_path, args.vehicle_model, args.scenario, seed
        )
        indexed_gains_output = indexed_path(
            gains_output_path, args.vehicle_model, args.scenario, seed
        )

        if not indexed_fuzzy.exists() or not indexed_pid.exists():
            print(f"Skipping seed {seed} as logs were not found.")
            continue

        indexed_output.parent.mkdir(parents=True, exist_ok=True)
        indexed_gains_output.parent.mkdir(parents=True, exist_ok=True)
        plot_velocity_logs(
            indexed_fuzzy,
            indexed_pid,
            indexed_output,
            indexed_gains_output,
            False,
            plt,
            scenario=f"{args.scenario}, {args.vehicle_model} (seed {seed})",
        )
        plotted_count += 1

    if plotted_count > 0 and args.show:
        plt.show()
    print(f"Created plots for {plotted_count} simulation pair(s).")


if __name__ == "__main__":
    main()
