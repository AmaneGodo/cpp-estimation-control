import pandas as pd
from matplotlib import pyplot as plt
from pathlib import Path

DEFAULT_OUT = Path("docs")
# Always run from the project root "python3 tools/plot_log.py"

df = pd.read_csv("log.csv")

time = df["t(s)"]
meas_pos = df["meas_pos(m)"]
meas_vel = df["meas_vel(m/s)"]
est_pos = df["est_pos(m)"]
est_vel = df["est_vel(m/s)"]
u = df["u(m/s^2)"]

DEFAULT_OUT.mkdir(parents=True, exist_ok=True)
# plot positions
plt.figure(figsize=(10, 5))
plt.plot(time, meas_pos, label="Measured Position")
plt.plot(time, est_pos, label="Estimated Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Positional plot (Measured vs Estimated)")
plt.legend()
plt.grid()
out_path = DEFAULT_OUT / "Positional_Plot.png"
plt.savefig(out_path, dpi = 300)
plt.show()

# plot velocity
plt.figure(figsize=(10, 5))
plt.plot(time, meas_vel, label="Measured Velocity")
plt.plot(time, est_vel, label="Estimated Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity plot (Measured vs Estimated)")
plt.legend()
plt.grid()
out_path = DEFAULT_OUT / "Velocity_Plot.png"
plt.savefig(out_path, dpi = 300)
plt.show()

# plot control input
plt.figure(figsize=(10, 5))
plt.plot(time, u)
plt.xlabel("Time (s)")
plt.ylabel("Control Input (m/s^2)")
plt.title("Control Input plot.png")
plt.grid()
out_path = DEFAULT_OUT / "Control_Input_Plot.png"
plt.savefig(out_path, dpi = 300)
plt.show()