"""
Rehabilitation Elbow Exoskeleton - PID Controller
Day 2: PID controller with 2nd-order joint dynamics and disturbance rejection.
"""

import os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class PIDController:
    def __init__(self, kp=45.0, ki=12.0, kd=6.0, dt=0.01, max_torque=40.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_torque = max_torque
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        p_term = self.kp * error

        # Anti-windup integration
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -5.0, 5.0)
        i_term = self.ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        self.prev_error = error

        total_torque = p_term + i_term + d_term
        return np.clip(total_torque, -self.max_torque, self.max_torque)


def main():
    # 1. Load Data
    data_path = os.path.join("data", "rehabilitation_simulation_data.csv")
    if not os.path.exists(data_path):
        print(f"Error: File not found at {data_path}")
        return

    df = pd.read_csv(data_path)
    print(f"Loaded {len(df)} samples from {data_path}")
    print(f"Columns: {list(df.columns)}\n")

    time = df['time_s'].values
    target_angles = df['target_angle_deg'].values
    disturbances = df['resistance_torque_Nm'].values

    dt = time[1] - time[0] if len(time) > 1 else 0.01
    n = len(df)

    # 2. Plant Dynamics Parameters (Arm segment)
    # Dynamics: J * theta_ddot + B * theta_dot = Tau_motor - Tau_disturbance
    J = 0.25   # kg*m^2 (Inertia)
    B = 1.2    # N*m*s/rad (Viscous friction)

    controller = PIDController(kp=2.0, ki=5.0, kd=0.4, dt=dt, max_torque=40.0)

    # 3. Simulation Arrays
    actual_angles = np.zeros(n)
    actual_velocities = np.zeros(n)
    applied_torques = np.zeros(n)

    current_angle_deg = target_angles[0]
    current_vel_rad_s = 0.0

    # 4. Simulation Loop
    for i in range(n):
        target_deg = target_angles[i]
        error_deg = target_deg - current_angle_deg

        # Motor PID torque
        motor_torque = controller.compute(error_deg)

        # Disturbance and viscous friction damping
        dist_torque = disturbances[i]
        damping_torque = B * current_vel_rad_s

        # Net torque acting on the arm joint
        net_torque = motor_torque - dist_torque - damping_torque

        # Angular acceleration (rad/s^2)
        angular_acc_rad_s2 = net_torque / J

        # Integration (Euler method)
        current_vel_rad_s += angular_acc_rad_s2 * dt
        current_angle_deg += np.degrees(current_vel_rad_s * dt)

        # Store results
        actual_angles[i] = current_angle_deg
        actual_velocities[i] = np.degrees(current_vel_rad_s)
        applied_torques[i] = motor_torque

    # 5. Metrics
    tracking_errors = target_angles - actual_angles
    mae = np.mean(np.abs(tracking_errors))
    rmse = np.sqrt(np.mean(tracking_errors ** 2))
    max_error = np.max(np.abs(tracking_errors))

    print("=" * 45)
    print("      PID REHABILITATION TRACKING METRICS     ")
    print("=" * 45)
    print(f"Mean Absolute Error (MAE):      {mae:.3f} deg")
    print(f"Root Mean Square Error (RMSE):  {rmse:.3f} deg")
    print(f"Max Absolute Tracking Error:    {max_error:.3f} deg")
    print("=" * 45 + "\n")

    # 6. Plot & Save Performance
    os.makedirs("figures", exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # Subplot 1: Angle Tracking
    axes[0].plot(time, target_angles, 'r--', label='Target Trajectory (Passive ROM)', linewidth=2.0)
    axes[0].plot(time, actual_angles, 'b-', label='Exoskeleton Actual Angle', linewidth=1.5, alpha=0.85)
    axes[0].set_ylabel('Elbow Angle (deg)', fontsize=11, fontweight='bold')
    axes[0].set_title('Elbow Exoskeleton - PID Trajectory Tracking', fontsize=12, fontweight='bold')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Control Torque & Disturbance
    axes[1].plot(time, applied_torques, 'g-', label='Motor Control Torque (PID)', linewidth=1.4)
    axes[1].plot(time, disturbances, 'm--', label='Patient Resistance / Spasm Disturbance', linewidth=1.2, alpha=0.8)
    axes[1].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    axes[1].set_ylabel('Torque (N·m)', fontsize=11, fontweight='bold')
    axes[1].set_title('Control Effort & Spasm Disturbance Compensation', fontsize=12, fontweight='bold')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    output_path = os.path.join("figures", "control_performance.png")
    plt.savefig(output_path, dpi=200)
    plt.close()
    print(f"Control performance plot saved to: {output_path}")


if __name__ == "__main__":
    main()
