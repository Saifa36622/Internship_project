import numpy as np
from ur_analytic_ik import ur3e

eef_pose = np.identity(4)

# Create a rotation matrix for top-down orientation
# For UR robots, typically Z should point downward, X forward
X = np.array([1.0, 0.0, 0.0])    # Forward
Y = np.array([0.0, 1.0, 0.0])    # Left
Z = np.array([0.0, 0.0, -1.0])   # Down

# Verify the orientation is valid (orthonormal)
top_down_orientation = np.column_stack([X, Y, Z])
if not np.allclose(np.linalg.det(top_down_orientation), 1.0):
    print("Warning: Rotation matrix is not valid (determinant != 1)")

print("Top-down orientation:\n", top_down_orientation)

# Set translation within typical UR3e workspace
# UR3e typical reach is about 500mm (0.5m)
translation = np.array([0.3, 0.0, 0.4])  # Adjusted Z to be lower
print(f"Translation (meters): {translation}")

# Reference pose from your comment
# [0.2952944781180743, 0.07949493768251537, 0.6152721239626734, -1.0190271069968346, 1.1301881648882925, -0.043673465305185205]

eef_pose[:3, :3] = top_down_orientation
eef_pose[:3, 3] = translation

print("End-effector pose:\n", eef_pose, end="\n\n")

# Verify the transform is valid
if not np.allclose(np.linalg.det(eef_pose[:3, :3]), 1.0):
    print("Warning: Transform matrix is not valid!")

# Check if position is within typical workspace
pos = np.linalg.norm(translation)
if pos > 0.5:  # UR3e reach is approximately 0.5m
    print(f"Warning: Position {pos}m might be outside workspace!")

solutions = ur3e.inverse_kinematics(eef_pose)

print(f"Number of IK solutions found: {len(solutions)}")
if len(solutions) > 0:
    for i, sol in enumerate(solutions):
        # Convert to degrees for better readability
        sol_deg = np.degrees(sol)
        print(f"Solution {i+1} (degrees): {sol_deg}")
else:
    print("No solutions found. Possible reasons:")
    print("1. Position is outside workspace")
    print("2. Orientation cannot be reached")
    print("3. Singular configuration") 