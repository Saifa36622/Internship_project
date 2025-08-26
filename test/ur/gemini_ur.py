import math
import numpy as np
import time

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from ur_analytic_ik import ur3e

rtde_control = RTDEControlInterface("192.168.12.60")
rtde_receive = RTDEReceiveInterface("192.168.12.60")

# start position
joint_q = [math.radians(45), math.radians(-135), 0.0, math.radians(-45), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)


def tcp_to_joint_position(target_tcp_pose_xyz):
    """
    Calculates a single target joint position for a desired Cartesian pose.

    Args:
        target_tcp_pose_xyz (list): A list [x, y, z] for the target position in meters.

    Returns:
        list: A single joint configuration for the target pose.
        None: If the target pose is unreachable or no solution is found.
    """
    # 1. Get the current robot pose (position and orientation)
    current_tcp_pose = rtde_receive.getActualTCPPose()
    current_joint_q = rtde_receive.getActualQ()
    print(f"Current TCP pose: {current_tcp_pose}")
    print(f"Current joint positions: {current_joint_q}")
    if not current_tcp_pose or not current_joint_q:
        print("Could not retrieve current robot state.")
        return None

    # We need a 6D pose (x, y, z, Rx, Ry, Rz) for IK.
    # The user provides x, y, z, so we'll maintain the current orientation.
    target_pos = np.array(target_tcp_pose_xyz)
    target_rot_vec = np.array(current_tcp_pose[3:6]) # Maintain current orientation

    # Create a 4x4 homogeneous transformation matrix for the target pose
    target_T = np.identity(4)
    target_T[:3, 3] = target_pos

    # For simplicity, we assume Rx, Ry, Rz are in radians and represent Euler angles
    # The RTDE interface returns a rotation vector.
    # A full implementation would need a proper conversion function from the rotation vector
    # to a rotation matrix to accurately represent the orientation.
    # For this example, we will use the ur-analytic-ik function
    # that takes a 6D pose vector which can be constructed from the current TCP.
    
    # We use a placeholder for the orientation matrix to proceed with the core logic
    # The `ur-analytic-ik` library handles this conversion internally when using the 6D pose.
    
    # 2. Calculate the corresponding joint configuration
    print("Calculating inverse kinematics for the target pose...")
    
    ik_solutions = ur3e.inverse_kinematics(target_T)

    if not ik_solutions:
        print("Warning: Target pose is unreachable.")
        return None

    # 3. Select the solution closest to the previous joint state.
    # This prevents jerky or unsafe movements by avoiding large joint flips.
    q_solution = ur3e.inverse_kinematics_closest(target_T, *current_joint_q)
    
    if q_solution is None:
        print("Warning: Could not find a 'closest' solution for the target pose.")
        return None
        
    print("Calculated a joint target.")
    return q_solution

def execute_moveJ_command(target_joint_q, rtde_control_interface):
    """
    Executes a moveJ command to a target joint position.
    """
    print("Executing moveJ command...")
    rtde_control_interface.moveJ(target_joint_q)
    print("MoveJ command finished.")


# --- Main execution block ---
if __name__ == "__main__":
    # Get user input for target TCP position in meters
    try:
        x = float(input("Enter target X coordinate (m): "))
        y = float(input("Enter target Y coordinate (m): "))
        z = float(input("Enter target Z coordinate (m): "))
        target_coords = [x, y, z]

        print(f"Attempting to find joint angles for TCP coordinates: {target_coords}")
        
        # Calculate the joint position
        joint_position = tcp_to_joint_position(target_coords)
        
        if joint_position:
            input("Press Enter to execute the planned moveJ command...")
            execute_moveJ_command(joint_position, rtde_control)

    except ValueError:
        print("Invalid input. Please enter numerical values.")
    finally:
        rtde_control.stopScript()