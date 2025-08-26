import numpy as np
import math
import time
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import threading
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

class UR3eTCPController:
    def __init__(self, robot_ip="192.168.12.60", control_frequency=10.0):
        """
        Initialize UR3e TCP Controller using joint position control
        Based on the paper's control scheme but adapted for RTDE position control
        """
        self.robot_ip = robot_ip
        self.control_frequency = control_frequency
        self.dt = 1.0 / control_frequency
        
        # Initialize RTDE interfaces
        self.rtde_control = RTDEControlInterface(robot_ip)
        self.rtde_receive = RTDEReceiveInterface(robot_ip)
        
        # Control parameters (from paper - Eq. 21-22)
        self.position_gains = np.array([2.0, 2.0, 2.0])  # Kp for x, y, z
        self.orientation_gains = np.array([1.5, 1.5, 1.5])  # Kp for rx, ry, rz
        
        # Integration step size for position updates
        self.integration_step = 0.1  # How much to move per control cycle
        
        # UR3e joint limits (radians)
        self.joint_limits_lower = np.array([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
        self.joint_limits_upper = np.array([2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
        
        # Error tolerances for pose reaching
        self.position_tolerance = 0.005  # 5mm
        self.orientation_tolerance = np.radians(3)  # 3 degrees
        
        # Control state
        self.control_active = False
        self.control_thread = None
        
        # Target poses
        self.target_poses = []
        self.current_pose_index = 0
        
        # UR3e DH parameters (for Jacobian computation)
        self.dh_a = np.array([0, -0.24365, -0.21325, 0, 0, 0])
        self.dh_d = np.array([0.1519, 0, 0, 0.11235, 0.08535, 0.0819])
        self.dh_alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
    
    def get_current_tcp_pose(self):
        """Get current TCP pose [x, y, z, rx, ry, rz]"""
        return np.array(self.rtde_receive.getActualTCPPose())
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        return np.array(self.rtde_receive.getActualQ())
    
    def forward_kinematics(self, joint_angles):
        """
        Compute forward kinematics for UR3e
        Returns TCP pose [x, y, z, rx, ry, rz]
        """
        # This is a simplified version - you might want to use a more accurate FK
        q = joint_angles
        
        # Transformation matrices
        T01 = self.dh_transform(q[0], self.dh_d[0], self.dh_a[0], self.dh_alpha[0])
        T12 = self.dh_transform(q[1], self.dh_d[1], self.dh_a[1], self.dh_alpha[1])
        T23 = self.dh_transform(q[2], self.dh_d[2], self.dh_a[2], self.dh_alpha[2])
        T34 = self.dh_transform(q[3], self.dh_d[3], self.dh_a[3], self.dh_alpha[3])
        T45 = self.dh_transform(q[4], self.dh_d[4], self.dh_a[4], self.dh_alpha[4])
        T56 = self.dh_transform(q[5], self.dh_d[5], self.dh_a[5], self.dh_alpha[5])
        
        # Total transformation
        T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        
        # Extract position
        position = T06[:3, 3]
        
        # Extract orientation as rotation vector
        rotation_matrix = T06[:3, :3]
        r = R.from_matrix(rotation_matrix)
        orientation = r.as_rotvec()
        
        return np.concatenate([position, orientation])
    
    def dh_transform(self, theta, d, a, alpha):
        """Create DH transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return T
    
    def compute_jacobian_numerical(self, q, epsilon=1e-6):
        """
        Compute Jacobian matrix numerically
        Returns 6x6 Jacobian matrix
        """
        jacobian = np.zeros((6, 6))
        
        # Get current pose
        pose_center = self.forward_kinematics(q)
        
        # Compute partial derivatives numerically
        for i in range(6):
            q_plus = q.copy()
            q_minus = q.copy()
            
            q_plus[i] += epsilon
            q_minus[i] -= epsilon
            
            pose_plus = self.forward_kinematics(q_plus)
            pose_minus = self.forward_kinematics(q_minus)
            
            jacobian[:, i] = (pose_plus - pose_minus) / (2 * epsilon)
        
        return jacobian
    
    def compute_pose_error(self, target_pose, current_pose):
        """
        Compute pose error [position_error, orientation_error]
        Following paper's Eq. 16 for position and Eq. 18-20 for orientation
        """
        # Position error (Eq. 16 from paper)
        pos_error = target_pose[:3] - current_pose[:3]
        
        # Orientation error (simplified axis-angle approach from paper)
        target_orient = target_pose[3:]
        current_orient = current_pose[3:]
        
        # Convert to rotation matrices
        R_target = R.from_rotvec(target_orient).as_matrix()
        R_current = R.from_rotvec(current_orient).as_matrix()
        
        # Compute rotation needed (Eq. 18 from paper)
        R_error = R_target @ R_current.T
        
        # Extract axis-angle error
        r_error = R.from_matrix(R_error)
        orient_error = r_error.as_rotvec()
        
        return pos_error, orient_error
    
    def solve_inverse_kinematics_step(self, target_pose, current_q):
        """
        Solve one step of inverse kinematics using differential approach
        Based on paper's cQP formulation (Eq. 25) but simplified for position control
        """
        current_pose = self.forward_kinematics(current_q)
        
        # Compute pose errors
        pos_error, orient_error = self.compute_pose_error(target_pose, current_pose)
        
        # Compute desired velocities (Eq. 21-22 from paper)
        desired_linear_vel = self.position_gains * pos_error
        desired_angular_vel = self.orientation_gains * orient_error
        
        # Limit velocities for safety
        max_linear_vel = 0.1  # m/s
        max_angular_vel = 0.5  # rad/s
        
        linear_vel_norm = np.linalg.norm(desired_linear_vel)
        if linear_vel_norm > max_linear_vel:
            desired_linear_vel = desired_linear_vel * (max_linear_vel / linear_vel_norm)
        
        angular_vel_norm = np.linalg.norm(desired_angular_vel)
        if angular_vel_norm > max_angular_vel:
            desired_angular_vel = desired_angular_vel * (max_angular_vel / angular_vel_norm)
        
        # Combine into 6D velocity vector
        desired_twist = np.concatenate([desired_linear_vel, desired_angular_vel])
        
        # Get Jacobian
        J = self.compute_jacobian_numerical(current_q)
        
        # Solve for joint velocities using pseudoinverse (simplified cQP)
        # In the paper, this would be solved as a proper cQP with constraints
        try:
            # Add damping for numerical stability (damped least squares from paper)
            damping_factor = 0.01
            J_damped = J.T @ np.linalg.inv(J @ J.T + damping_factor * np.eye(6))
            joint_velocities = J_damped @ desired_twist
        except:
            # Fallback to simple pseudoinverse
            joint_velocities = np.linalg.pinv(J) @ desired_twist
        
        # Integrate to get new joint positions
        new_joint_positions = current_q + joint_velocities * self.integration_step
        
        # Apply joint limits
        new_joint_positions = np.clip(new_joint_positions, 
                                     self.joint_limits_lower, 
                                     self.joint_limits_upper)
        
        return new_joint_positions, pos_error, orient_error
    
    def add_target_pose(self, position, orientation_rpy=None, orientation_matrix=None):
        """
        Add target pose to the sequence
        position: [x, y, z] in meters
        orientation_rpy: [rx, ry, rz] in radians OR
        orientation_matrix: 3x3 rotation matrix
        """
        if orientation_matrix is not None:
            r = R.from_matrix(orientation_matrix)
            orientation_rpy = r.as_rotvec()
        elif orientation_rpy is None:
            orientation_rpy = [0, 0, 0]
        
        pose = np.concatenate([position, orientation_rpy])
        self.target_poses.append(pose)
    
    def is_pose_reached(self, target_pose, current_pose):
        """Check if target pose is reached within tolerances"""
        pos_error, orient_error = self.compute_pose_error(target_pose, current_pose)
        
        position_reached = np.linalg.norm(pos_error) < self.position_tolerance
        orientation_reached = np.linalg.norm(orient_error) < self.orientation_tolerance
        
        return position_reached and orientation_reached
    
    def control_loop(self):
        """
        Main control loop implementing the paper's TCP control scheme
        Using joint position commands instead of velocities
        """
        print("Starting TCP control loop with joint position control...")
        
        pose_hold_start = None
        hold_duration = 2.0  # Hold pose for 2 seconds before moving to next
        
        while self.control_active and self.current_pose_index < len(self.target_poses):
            loop_start_time = time.time()
            
            # Get current state
            current_joint_q = self.get_current_joint_positions()
            current_tcp_pose = self.get_current_tcp_pose()
            target_pose = self.target_poses[self.current_pose_index]
            
            # Check if current pose is reached
            pose_reached = self.is_pose_reached(target_pose, current_tcp_pose)
            
            if pose_reached:
                if pose_hold_start is None:
                    pose_hold_start = time.time()
                    print(f"Pose {self.current_pose_index + 1} reached! Holding...")
                elif time.time() - pose_hold_start >= hold_duration:
                    print(f"Moving to pose {self.current_pose_index + 2}")
                    self.current_pose_index += 1
                    pose_hold_start = None
                    continue
            else:
                pose_hold_start = None
                
                # Compute next joint positions
                next_joint_q, pos_error, orient_error = self.solve_inverse_kinematics_step(
                    target_pose, current_joint_q)
                
                # Print debug info
                pos_error_norm = np.linalg.norm(pos_error)
                orient_error_norm = np.linalg.norm(orient_error)
                print(f"Pose {self.current_pose_index + 1}: "
                      f"Pos error: {pos_error_norm:.4f}m, "
                      f"Orient error: {orient_error_norm:.4f}rad")
                
                # Send joint position command
                try:
                    # Use moveJ with small time to make it more responsive
                    self.rtde_control.moveJ(next_joint_q.tolist(), 
                                          speed=1.0, 
                                          acceleration=2.0, 
                                          asynchronous=True)
                except Exception as e:
                    print(f"Failed to send joint command: {e}")
                    break
            
            # Maintain control frequency
            elapsed_time = time.time() - loop_start_time
            sleep_time = self.dt - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("TCP control loop finished")
    
    def start_control(self):
        """Start the control loop"""
        if not self.target_poses:
            print("No target poses defined!")
            return
        
        self.control_active = True
        self.current_pose_index = 0
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
    
    def stop_control(self):
        """Stop the control loop"""
        self.control_active = False
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
    
    def shutdown(self):
        """Shutdown the controller"""
        self.stop_control()
        self.rtde_control.stopScript()

def main():
    """Example usage with pick-and-place trajectory"""
    # Initialize controller
    controller = UR3eTCPController("192.168.12.60", control_frequency=10.0)
    
    # Move to safe starting position
    print("Moving to initial position...")
    initial_joints = [math.radians(45), math.radians(-135), 0.0, 
                     math.radians(-45), math.radians(90), math.radians(45)]
    controller.rtde_control.moveJ(initial_joints)
    time.sleep(3)
    
    # Define pick-and-place trajectory similar to paper's Table 2
    print("Defining trajectory poses...")
    
    # Pose 1: Hover above pickup location
    controller.add_target_pose([0.4, -0.1, 0.3], 
                              orientation_rpy=[0, math.pi, 0])
    
    # # Pose 2: Lower to pickup
    # controller.add_target_pose([0.4, -0.1, 0.15], 
    #                           orientation_rpy=[0, math.pi, 0])
    
    # # Simulate gripper close
    # print("Add gripper close command here")
    
    # # Pose 3: Lift up after pickup
    # controller.add_target_pose([0.4, -0.1, 0.35], 
    #                           orientation_rpy=[0, math.pi, 0])
    
    # # Pose 4: Move to place location
    # controller.add_target_pose([0.1, 0.3, 0.35], 
    #                           orientation_rpy=[0, math.pi, 0])
    
    # # Pose 5: Lower to place
    # controller.add_target_pose([0.1, 0.3, 0.15], 
    #                           orientation_rpy=[0, math.pi, 0])
    
    # # Simulate gripper open
    # print("Add gripper open command here")
    
    # # Pose 6: Retract after placing
    # controller.add_target_pose([0.1, 0.3, 0.35], 
    #                           orientation_rpy=[0, math.pi, 0])
    
    # # Pose 7: Return to home position
    # controller.add_target_pose([0.3, 0.0, 0.4], 
    #                           orientation_rpy=[0, math.pi, 0])
    
    try:
        print("Starting TCP control...")
        controller.start_control()
        
        # Monitor progress
        while controller.control_active and controller.current_pose_index < len(controller.target_poses):
            time.sleep(1)
            print(f"Current pose: {controller.current_pose_index + 1}/{len(controller.target_poses)}")
        
        print("Trajectory completed!")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.shutdown()
        print("Controller shutdown complete")

if __name__ == "__main__":
    main()