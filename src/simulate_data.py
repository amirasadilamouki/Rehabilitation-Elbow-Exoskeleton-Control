"""
Rehabilitation Elbow Exoskeleton Simulation - Day 1
Generate simulated rehabilitation data.
"""

import os

import numpy as np
import pandas as pd


def generate_rehabilitation_data(duration=10.0, dt=0.01):
    """
    Generate target elbow angle, current angle, and external resistance.
    """

    time = np.arange(0.0, duration, dt)

    # Target movement: smooth trajectory between 0 and 90 degrees
    target_angle = 45.0 * (1.0 - np.cos(2.0 * np.pi * 0.2 * time))

    # Initial tracking error: current angle starts below the target
    initial_tracking_error = 8.0 * np.exp(-0.35 * time)

    # Muscle resistance torque
    baseline_resistance = 0.8 + 0.25 * np.sin(2.0 * np.pi * 0.15 * time)

    # Short muscle-spasm event around t = 4 seconds
    spasm_resistance = 1.8 * np.exp(-0.5 * ((time - 4.0) / 0.18) ** 2)

    # Small deterministic disturbance
    measurement_disturbance = 0.08 * np.sin(2.0 * np.pi * 1.2 * time)

    resistance_torque = (
        baseline_resistance
        + spasm_resistance
        + measurement_disturbance
    )

    # Simulated current angle before applying the Day 2 controller
    current_angle = target_angle - initial_tracking_error

    data = pd.DataFrame(
        {
            "time_s": time,
            "target_angle_deg": target_angle,
            "current_angle_deg": current_angle,
            "resistance_torque_Nm": resistance_torque,
        }
    )

    return data


def main():
    data = generate_rehabilitation_data()

    output_directory = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        "data",
    )

    os.makedirs(output_directory, exist_ok=True)

    output_path = os.path.join(
        output_directory,
        "rehabilitation_simulation_data.csv",
    )

    data.to_csv(output_path, index=False)

    print("Simulation completed successfully.")
    print(f"Rows generated: {len(data)}")
    print(f"Output file: {output_path}")
    
if __name__ == "__main__":
    main()


