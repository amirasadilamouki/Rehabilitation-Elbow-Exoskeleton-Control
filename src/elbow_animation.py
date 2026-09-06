import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

os.makedirs("figures", exist_ok=True)

# 1. Simulation Time & Reference Data (Matching Day 2 dynamics)
t = np.linspace(0, 10, 300)
# Target: 0 to 90 degrees flexion/extension cycle
target = 45.0 * (1.0 - np.cos(2 * np.pi * t / 5.0))

# Actual tracking with minor natural lag and spasm at t=4s
actual = target.copy()
# Smooth disturbance (spasm) at t=4s
spasm_dist = 4.0 * np.exp(-((t - 4.0) / 0.3) ** 2)
actual = actual - spasm_dist + 0.3 * np.sin(4 * np.pi * t)

# Motor torque profile
torque = 1.2 * np.sin(2 * np.pi * t / 5.0) + 0.8 * (actual - target) + 0.5 * spasm_dist

# 2. Figure Layout
fig = plt.figure(figsize=(13, 6))
grid = fig.add_gridspec(1, 2, width_ratios=[1.1, 1.4])

ax_arm = fig.add_subplot(grid[0, 0])
ax_plot = fig.add_subplot(grid[0, 1])

# Kinematic parameters
L_upper = 1.5   # Upper arm length
L_fore = 1.3    # Forearm length
shoulder = np.array([0.0, 1.2])
elbow = shoulder + np.array([0.0, -L_upper])  # Upper arm hangs vertically downwards

# Configure Left Subplot (Exoskeleton Biomechanics)
ax_arm.set_xlim(-1.8, 1.8)
ax_arm.set_ylim(-1.8, 1.8)
ax_arm.set_aspect("equal")
ax_arm.set_title("Elbow Exoskeleton Motion Simulation", fontsize=12, fontweight='bold')
ax_arm.set_xlabel("X Position (m)")
ax_arm.set_ylabel("Y Position (m)")
ax_arm.grid(True, linestyle=":", alpha=0.6)

# Static Shoulder Joint
ax_arm.plot(shoulder[0], shoulder[1], "s", color="darkgray", markersize=10, label="Shoulder (Fixed)")

# Arm Link artists
upper_arm_line, = ax_arm.plot([], [], color="#2b5c8f", linewidth=8, solid_capstyle='round', label="Upper Arm (Fixed)")
forearm_line, = ax_arm.plot([], [], color="#2ca02c", linewidth=7, solid_capstyle='round', label="Forearm / Exoskeleton")
joint_elbow, = ax_arm.plot([], [], "o", color="#d62728", markersize=11, label="Elbow Actuator")
wrist_marker, = ax_arm.plot([], [], "o", color="#1f77b4", markersize=7)

info_text = ax_arm.text(0.04, 0.05, "", transform=ax_arm.transAxes, fontsize=10,
                        verticalalignment='bottom', bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.85, edgecolor='gray'))

ax_arm.legend(loc="upper left", fontsize=8, framealpha=0.85)

# Configure Right Subplot (Tracking & Control curves)
ax_plot.set_xlim(0, 10)
ax_plot.set_ylim(-10, 105)
ax_plot.set_title("Real-Time Joint Angle Tracking", fontsize=12, fontweight='bold')
ax_plot.set_xlabel("Time (s)")
ax_plot.set_ylabel("Elbow Flexion Angle (deg)")
ax_plot.grid(True, linestyle=":", alpha=0.6)

line_target, = ax_plot.plot([], [], "r--", linewidth=2, label="Target Trajectory", zorder=5)
line_actual, = ax_plot.plot([], [], "b-", linewidth=2, alpha=0.7, label="Actual Angle")

current_marker, = ax_plot.plot([], [], "o", color="purple", markersize=8, label="Current State")

ax_plot.legend(loc="upper right", fontsize=9)

def init():
    upper_arm_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
    forearm_line.set_data([], [])
    joint_elbow.set_data([elbow[0]], [elbow[1]])
    wrist_marker.set_data([], [])
    line_target.set_data([], [])
    line_actual.set_data([], [])
    current_marker.set_data([], [])
    info_text.set_text("")
    return upper_arm_line, forearm_line, joint_elbow, wrist_marker, line_target, line_actual, current_marker, info_text

def update(frame):
    # Angle in radians (0 deg = straight down, 90 deg = horizontal forward flexion)
    theta_deg = actual[frame]
    theta_rad = np.deg2rad(theta_deg)
    
    # Forearm position: rotating around elbow
    # Angle measured from vertical hanging position (-pi/2) forward towards horizontal (0)
    angle = -np.pi/2 + theta_rad
    wrist = elbow + np.array([L_fore * np.cos(angle), L_fore * np.sin(angle)])
    
    # Update arm components
    upper_arm_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
    forearm_line.set_data([elbow[0], wrist[0]], [elbow[1], wrist[1]])
    joint_elbow.set_data([elbow[0]], [elbow[1]])
    wrist_marker.set_data([wrist[0]], [wrist[1]])
    
    # Update tracking graph
    line_target.set_data(t[:frame + 1], target[:frame + 1])
    line_actual.set_data(t[:frame + 1], actual[:frame + 1])
    current_marker.set_data([t[frame]], [actual[frame]])
    
    info_text.set_text(
        f"Time: {t[frame]:.2f} s\n"
        f"Flexion Angle: {actual[frame]:.1f}°\n"
        f"Target Angle: {target[frame]:.1f}°\n"
        f"Control Torque: {torque[frame]:.2f} N·m"
    )
    
    return upper_arm_line, forearm_line, joint_elbow, wrist_marker, line_target, line_actual, current_marker, info_text

ani = FuncAnimation(fig, update, frames=len(t), init_func=init, interval=30, blit=True)

plt.tight_layout()

# Save final state preview
update(len(t) - 1)
plt.savefig("figures/elbow_animation.png", dpi=300, bbox_inches="tight")
print("✅ Animation preview successfully saved to figures/elbow_animation.png")

plt.show()
