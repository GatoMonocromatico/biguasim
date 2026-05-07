import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Rotor positions in ENU (y, x, z)
rotor_pos = {
    'r1': np.array([0.22, 0.22, 0.08]),          # Rotor 1 position
            'r2': np.array([0.22, -0.22, 0.08]),         # Rotor 2 position
            'r3': np.array([-0.22, 0.22, 0.08]),         # Rotor 3 position
            'r4': np.array([-0.22, -0.22, 0.08]),        # Rotor 4 position
            'r5': np.array([0.156, 0.111, 0.045]),       # Rotor 5 position
            'r6': np.array([0.156, -0.111, 0.045]),      # Rotor 6 position
            'r7': np.array([-0.156, 0.111, 0.045]),      # Rotor 7 position
            'r8': np.array([-0.156, -0.111, 0.045]),     # Rotor 8 position
            
}

# Rotor direction vectors (x, y, z) -> must be converted to (y, x, z) for ENU
# rotor_directions = np.array([
#     [0, 0, 1],
#     [0, 0, 1],
#     [np.cos(np.pi/4), np.sin(np.pi/4), 0],
#     [np.cos(3*np.pi/4), np.sin(3*np.pi/4), 0],
#     [np.cos(7*np.pi/4), np.sin(7*np.pi/4), 0],
#     [np.cos(5*np.pi/4), np.sin(5*np.pi/4), 0],
# ])
rotor_directions = np.array([[0, 0, 1],
                            [0, 0, 1],
                            [0, 0, 1],
                            [0, 0, 1],
                            [np.cos(7*np.pi/4), np.sin(7*np.pi/4), 0],
                            [np.cos(np.pi/4), np.sin(np.pi/4), 0],
                            [np.cos(5*np.pi/4), np.sin(5*np.pi/4), 0],
                            [np.cos(3*np.pi/4), np.sin(3*np.pi/4), 0]])

# Convert direction vectors from (x, y, z) to (y, x, z)
rotor_dirs_enu = np.array([[v[0], v[1], v[2]] for v in rotor_directions])

# Plot setup
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Rotor Setup in ENU Frame (y=East, x=North, z=Up)")
ax.set_xlabel('East (y)')
ax.set_ylabel('North (x)')
ax.set_zlabel('Up (z)')

# Plot each rotor and its direction
for i, (key, pos) in enumerate(rotor_pos.items()):
    dir_vec = rotor_dirs_enu[i]
    ax.scatter(pos[0], pos[1], pos[2], label=key, s=50)
    ax.quiver(pos[0], pos[1], pos[2],
              dir_vec[0]*0.1, dir_vec[1]*0.1, dir_vec[2]*0.1,
              color='r', arrow_length_ratio=0.2)

# Add body frame axes (vehicle reference)
ax.quiver(0, 0, 0, 0.2, 0, 0, color='r', linewidth=2, label='x: Front')
ax.quiver(0, 0, 0, 0, 0.2, 0, color='g', linewidth=2, label='y: Left')
ax.quiver(0, 0, 0, 0, 0, 0.2, color='b', linewidth=2, label='z: Up')

# Adjust aspect ratio and grid
ax.set_box_aspect([1,1,0.5])
ax.legend()
ax.grid(True)
plt.show()

