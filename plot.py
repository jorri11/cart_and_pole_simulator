import matplotlib
matplotlib.use("TkAgg")  # Force GUI backend

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("data.csv")

# Convert radians to degrees
df["pole_angle_deg"] = np.degrees(df["pole_angle"])

# Create figure with 2 subplots
fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Plot cart position
axs[0].plot(df["time"], df["cart_pos"])
axs[0].set_title("Cart Position vs Time")
axs[0].set_ylabel("Cart Position")
axs[0].grid(True)

# Plot pole angle in degrees
axs[1].plot(df["time"], df["pole_angle_deg"])
axs[1].set_title("Pole Angle vs Time")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Pole Angle (degrees)")
axs[1].grid(True)

plt.tight_layout()
plt.show()