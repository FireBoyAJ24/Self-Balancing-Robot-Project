import pandas as pd
import numpy as np
import re

# Load the data
df = pd.read_csv("pid_data.csv")

# Time column
time = df["Time (s)"]

# Store metrics
results = []

for col in df.columns:
    if col.startswith("Pitch ("):
        # Extract PID constants
        match = re.search(r"Kp: ([\d.e+\-]+), Ki: ([\d.e+\-]+), Kd: ([\d.e+\-]+)", col)
        if match:
            kp, ki, kd = map(float, match.groups())
            
            pitch = df[col]
            abs_error = np.abs(pitch)
            iae = np.trapz(abs_error, time)  # Integral of Absolute Error

            results.append({
                "Kp": kp,
                "Ki": ki,
                "Kd": kd,
                "IAE": iae,
                "Final Pitch": abs(pitch.iloc[-1]),
                "Max Overshoot": max(abs(pitch))
            })

# Create summary DataFrame
summary = pd.DataFrame(results)

# Sort by lowest IAE
best = summary.sort_values("IAE").reset_index(drop=True)

print(best.head(10))  # Top 10 PID sets

# Save for reference
best.to_csv("pid_ranking.csv", index=False)
