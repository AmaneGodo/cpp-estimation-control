import sys
import pandas as pd
from matplotlib import pyplot as plt
from pathlib import Path

# Usage:
#   python tools/plot_log.py log_bias.csv
#   python tools/plot_log.py log_nominal.csv
#
# If no argument is provided, defaults to log_bias.csv.
#
# Always run from project root:
#   python tools/plot_log.py ...

DEFAULT_OUT = Path("docs")
# Always run from the project root "python3 tools/plot_log.py"

def save_show(fig_name: str, out_dir: Path) -> None:
    out_path = out_dir / fig_name
    plt.savefig(out_path, dpi = 300)
    plt.show()
    plt.close()

def main() -> int:
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path ("log_bias.csv")
    if not csv_path.exists():
        print(f"ERROR: CSV not found: {csv_path}")
        return 1

    df = pd.read_csv(csv_path)
    cols = set(df.columns)

    # required - must be included in both csv
    required = {
        "t(s)", 
        "meas_pos(m)",
        "meas_vel(m/s)",
        "est_pos(m)", 
        "est_vel(m/s)", 
        "u(m/s^2)",
    }

    missing = required - cols
    if missing:
        print(f"ERROR: Missing required columns in {csv_path.name}: {sorted(missing)}")
        print(f"Columns found: {list(df.columns)}")
        return 1

    # Detect bias mode by presense of bias columns
    has_est_bias = "est_bias(m/s^2)" in cols
    has_true_bias = "true_bias(m/s^2)" in cols
    bias_mode = has_est_bias or has_true_bias

    # create output folder
    out_dir = DEFAULT_OUT / ("bias" if bias_mode else "nominal")

    out_dir.mkdir(parents=True, exist_ok=True)

    time = df["t(s)"]
    meas_pos = df["meas_pos(m)"]
    meas_vel = df["meas_vel(m/s)"]
    est_pos = df["est_pos(m)"]
    est_vel = df["est_vel(m/s)"]
    u = df["u(m/s^2)"]

    # plot positions
    plt.figure(figsize=(10, 5))
    plt.plot(time, meas_pos, label="Measured Position")
    plt.plot(time, est_pos, label="Estimated Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title(f"Positional plot (Measured vs Estimated) - {csv_path.name}")
    plt.legend()
    plt.grid()
    save_show("Positional_Plot.png", out_dir)

    # plot velocity
    plt.figure(figsize=(10, 5))
    plt.plot(time, meas_vel, label="Measured Velocity")
    plt.plot(time, est_vel, label="Estimated Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title(f"Velocity plot (Measured vs Estimated) - {csv_path.name}")
    plt.legend()
    plt.grid()
    save_show("Velocity_Plot.png", out_dir)

    # plot bias
    if bias_mode:
        plt.figure(figsize=(10, 5))

        if has_true_bias:
            plt.plot(time, df["true_bias(m/s^2)"], label = "True bias")

        if has_est_bias:
            plt.plot(time, df["est_bias(m/s^2)"], label = "Estimated Bias")
    

        plt.xlabel("Time(s)")
        plt.ylabel("Acceleration (m/s^2)")
        plt.title(f"Acceleration Bias plot - {csv_path.name}")
        plt.legend()
        plt.grid()
        save_show("Acceleration_Bias_Plot.png", out_dir)

    else:
        print("Note: bias data was not found; skipping bias plot.")

    # plot control input
    plt.figure(figsize=(10, 5))
    plt.plot(time, u)
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input (m/s^2)")
    plt.title(f"Control Input plot.png - {csv_path.name}")
    plt.grid()
    save_show("Control_input_Plot.png", out_dir)

    print(f"Saved plots to: {out_dir}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())