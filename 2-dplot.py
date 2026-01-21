import matplotlib.pyplot as plt
import numpy as np

# Create figure
fig, ax = plt.subplots(figsize=(10, 8))

# Set limits
ax.set_xlim(-0.1, 0.4)
ax.set_ylim(-0.1, 0.4)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.7)

# Robot parameters
L1 = 0.15
L2 = 0.15
theta1 = np.radians(45)  # 45 degrees
theta2 = np.radians(30)  # 30 degrees

# Calculate positions
x1 = L1 * np.cos(theta1)
y1 = L1 * np.sin(theta1)
x2 = x1 + L2 * np.cos(theta1 + theta2)
y2 = y1 + L2 * np.sin(theta1 + theta2)

# Draw links
ax.plot([0, x1], [0, y1], 'b-', linewidth=3, label=f'Link 1 (L₁={L1}m)')
ax.plot([x1, x2], [y1, y2], 'r-', linewidth=3, label=f'Link 2 (L₂={L2}m)')

# Draw joints
ax.plot(0, 0, 'ko', markersize=10, label='Base (Joint 1)')
ax.plot(x1, y1, 'ko', markersize=8, label='Joint 2')
ax.plot(x2, y2, 'go', markersize=10, label='End Effector')

# Draw angles
# θ₁ arc
theta1_arc = np.linspace(0, theta1, 50)
ax.plot(0.05*np.cos(theta1_arc), 0.05*np.sin(theta1_arc), 'k-')
ax.text(0.07, 0.03, r'$\theta_1$', fontsize=12)

# θ₂ arc
theta2_arc = np.linspace(theta1, theta1+theta2, 50)
ax.plot(x1 + 0.05*np.cos(theta2_arc), y1 + 0.05*np.sin(theta2_arc), 'k-')
ax.text(x1+0.07, y1+0.03, r'$\theta_2$', fontsize=12)

# Labels
ax.text(0, -0.02, 'Base', ha='center', fontsize=10)
ax.text(x1/2, y1/2, f'L₁={L1}m\nm₁, l₁', ha='center', fontsize=9)
ax.text((x1+x2)/2, (y1+y2)/2, f'L₂={L2}m\nm₂, l₂', ha='center', fontsize=9)
ax.text(x2, y2+0.02, 'End Effector\n(x, y)', ha='center', fontsize=10)

# Coordinate system
ax.arrow(0, 0, 0.3, 0, head_width=0.01, head_length=0.01, fc='black')
ax.arrow(0, 0, 0, 0.3, head_width=0.01, head_length=0.01, fc='black')
ax.text(0.32, 0, 'X', fontsize=12)
ax.text(0, 0.32, 'Y', fontsize=12)

# Gravity
ax.arrow(0.2, 0.3, 0, -0.1, head_width=0.01, head_length=0.01, fc='red')
ax.text(0.21, 0.25, 'g', color='red', fontsize=12)

# Title
plt.title('2-DOF Planar Robot Arm', fontsize=14, fontweight='bold')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.legend(loc='upper left')
plt.tight_layout()

# Save as image
plt.savefig('2dof_robot_arm.png', dpi=300, bbox_inches='tight')
plt.show()
