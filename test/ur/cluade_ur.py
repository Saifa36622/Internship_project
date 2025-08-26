import numpy as np
import math
import time
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import threading
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

# rtde_control = RTDEControlInterface("192.168.12.60")

class UR3eTCPController:
    def __init__(self, robot_ip="192.168.12.60", sample_time=0.01):
        """
        Initialize UR3e TCP Controller based on the paper's control scheme
        """
        self.robot_ip = robot_ip
        self.sample_time = sample_time
        
        # Initialize RTDE interfaces
        self.rtde_control = RTDEControlInterface(robot_ip)
        self.rtde_receive = RTDEReceiveInterface(robot_ip)
        
        # Control parameters (from paper)
        self.p_gains = np.array([1.0, 1.0, 1.0])  # Position gains px, py, pz
        self.p_theta_gain = 1.0  # Orientation gain
        
        # Velocity limits (m/s and rad/s)
        self.max_linear_vel = 0.25
        self.max_angular_vel = 1.0
        
        # Joint limits (UR3e specific)
        self.joint_limits_lower = np.array([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
        self.joint_limits_upper = np.array([2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
        self.joint_vel_limits = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])  # rad/s
        
        # Error margins for pose switching
        self.position_error_margin = 0.005  # 5mm
        self.orientation_error_margin = np.radians(2)  # 2 degrees
        
        # Control state
        self.current_target_pose = None
        self.control_active = False
        self.control_thread = None
        
        # Target poses lookup table
        self.target_poses = []
        self.current_pose_index = 0
        
    def get_tcp_pose(self):
        """Get current TCP pose from robot"""
        tcp_pose = self.rtde_receive.getActualTCPPose()
        position = np.array(tcp_pose[:3])
        orientation = np.array(tcp_pose[3:])  # rx, ry, rz
        return position, orientation
    
    def get_joint_positions(self):
        """Get current joint positions"""
        return np.array(self.rtde_receive.getActualQ())
    
    def get_joint_velocities(self):
        """Get current joint velocities"""
        return np.array(self.rtde_receive.getActualQd())
    
    def compute_jacobian(self, q):
        """
        Compute Jacobian matrix for UR3e
        Simplified version - in practice, you'd use the robot's built-in Jacobian
        """
        # This is a simplified Jacobian computation
        # For UR3e, you should use the robot's actual DH parameters
        tcp_pose = self.rtde_receive.getActualTCPPose()
        
        # Get Jacobian from robot (if available) or compute analytically
        # For now, using a numerical approximation
        J = np.zeros((6, 6))
        
        # Numerical differentiation for Jacobian
        delta = 1e-6
        q_current = self.get_joint_positions()
        
        for i in range(6):
            q_plus = q_current.copy()
            q_minus = q_current.copy()
            q_plus[i] += delta
            q_minus[i] -= delta
            
            # This would need forward kinematics implementation
            # For now, using a simplified approach
            J[:3, i] = np.random.rand(3) * 0.1  # Placeholder
            J[3:, i] = np.random.rand(3) * 0.1  # Placeholder
            
        return J
    
    def axis_angle_error(self, R_des, R_curr):
        """
        Compute orientation error using axis-angle representation (from paper)
        """
        R_need = R_des.T @ R_curr
        
        # Extract axis-angle error
        trace_R = np.trace(R_need)
        theta = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
        
        if abs(theta) < 1e-6:
            return 0.0
            
        return theta
    
    def compute_desired_velocities(self, pos_error, orient_error):
        """
        Compute desired TCP velocities from errors (Eq. 21-22 from paper)
        """
        # Linear velocities
        v_des = self.p_gains * pos_error
        
        # Limit linear velocities
        v_norm = np.linalg.norm(v_des)
        if v_norm > self.max_linear_vel:
            v_des = v_des * (self.max_linear_vel / v_norm)
        
        # Angular velocity
        omega_des = self.p_theta_gain * orient_error
        
        # Limit angular velocity
        if abs(omega_des) > self.max_angular_vel:
            omega_des = np.sign(omega_des) * self.max_angular_vel
        
        return v_des, omega_des
    
    def solve_cqp_inverse_kinematics(self, v_des, omega_des):
        """
        Solve constrained quadratic program for inverse kinematics (Eq. 25 from paper)
        This is a simplified version of the cQP from the paper
        """
        q_current = self.get_joint_positions()
        qd_current = self.get_joint_velocities()
        
        # Get Jacobian
        J = self.compute_jacobian(q_current)
        J_pos = J[:3, :]  # Position Jacobian
        J_orient = J[3:, :]  # Orientation Jacobian
        
        # Desired task space velocity
        xd_des = np.concatenate([v_des, [omega_des, 0, 0]])  # Simplified
        
        # Quadratic programming problem (simplified)

        # min 0.5 * qd^T * W * qd + 0.5 * ||J*qd - xd_des||^2
        
        def objective(qd):
            # Regularization term
            reg_term = 0.5 * np.sum(qd**2)
            
            # Task space error term
            J_full = J[:4, :]  # Using first 4 rows (3 pos + 1 orient)
            xd_des_reduced = xd_des[:4]
            task_error = J_full @ qd - xd_des_reduced
            task_term = 0.5 * np.sum(task_error**2)
            
            return reg_term + 10.0 * task_term  # Weight task space higher
        
        # Constraints
        def constraint_joint_limits(qd):
            q_next = q_current + qd * self.sample_time
            return np.concatenate([
                q_next - self.joint_limits_lower,
                self.joint_limits_upper - q_next
            ])
        
        def constraint_vel_limits(qd):
            return np.concatenate([
                qd + self.joint_vel_limits,
                self.joint_vel_limits - qd
            ])
        
        # Setup constraints
        constraints = [
            {'type': 'ineq', 'fun': constraint_joint_limits},
            {'type': 'ineq', 'fun': constraint_vel_limits}
        ]
        
        # Initial guess
        x0 = np.zeros(6)
        
        # Solve optimization
        try:
            result = minimize(objective, x0, method='SLSQP', constraints=constraints)
            if result.success:
                return result.x
            else:
                print("Optimization failed, using zero velocities")
                return np.zeros(6)
        except:
            print("Exception in optimization, using zero velocities")
            return np.zeros(6)
    
    def add_target_pose(self, position, orientation_matrix=None, orientation_rpy=None):
        """
        Add target pose to lookup table
        position: [x, y, z] in meters
        orientation_matrix: 3x3 rotation matrix OR
        orientation_rpy: [rx, ry, rz] in radians
        """
        if orientation_matrix is not None:
            # Convert rotation matrix to axis-angle
            r = R.from_matrix(orientation_matrix)
            orientation_rpy = r.as_rotvec()
        elif orientation_rpy is None:
            # Default orientation (pointing down)
            orientation_rpy = [0, 0, 0]
        
        pose = {
            'position': np.array(position),
            'orientation': np.array(orientation_rpy)
        }
        self.target_poses.append(pose)
    
    def check_pose_reached(self, target_pose):
        """Check if target pose is reached within error margins"""
        current_pos, current_orient = self.get_tcp_pose()
        
        # Position error
        pos_error = np.linalg.norm(target_pose['position'] - current_pos)
        
        # Orientation error (simplified)
        orient_error = np.linalg.norm(target_pose['orientation'] - current_orient)
        
        position_reached = pos_error < self.position_error_margin
        orientation_reached = orient_error < self.orientation_error_margin
        
        return position_reached and orientation_reached, pos_error, orient_error
    
    def control_loop(self):
        """Main control loop implementing the paper's control scheme"""
        print("Starting TCP control loop...")
        
        pose_reached_time = None
        required_hold_time = 1.0  # Hold position for 1 second before switching
        
        while self.control_active and self.current_pose_index < len(self.target_poses):
            start_time = time.time()
            
            # Get current target pose
            target_pose = self.target_poses[self.current_pose_index]
            
            # Get current TCP pose
            current_pos, current_orient = self.get_tcp_pose()
            
            # Compute errors (Eq. 16 from paper)
            pos_error = target_pose['position'] - current_pos
            
            # Simplified orientation error
            orient_error_vector = target_pose['orientation'] - current_orient
            orient_error = np.linalg.norm(orient_error_vector)
            
            # Check if pose is reached
            pose_reached, pos_err_norm, orient_err_norm = self.check_pose_reached(target_pose)
            
            print(f"Pose {self.current_pose_index}: Pos error: {pos_err_norm:.4f}m, Orient error: {orient_err_norm:.4f}rad")
            
            if pose_reached:
                if pose_reached_time is None:
                    pose_reached_time = time.time()
                elif time.time() - pose_reached_time > required_hold_time:
                    print(f"Pose {self.current_pose_index} reached! Moving to next pose.")
                    self.current_pose_index += 1
                    pose_reached_time = None
                    continue
            else:
                pose_reached_time = None
            
            # Compute desired velocities (Eq. 21-22 from paper)
            v_des, omega_des = self.compute_desired_velocities(pos_error, orient_error)
            
            # Solve cQP for joint velocities (Eq. 25 from paper)
            qd_desired = self.solve_cqp_inverse_kinematics(v_des, omega_des)
            
            # Send velocity command to robot
            try:
                self.rtde_control.speedJ(qd_desired.tolist(), acceleration=1.0, time=self.sample_time*2)
            except:
                print("Failed to send velocity command")
                break
            
            # Maintain sample time
            elapsed = time.time() - start_time
            if elapsed < self.sample_time:
                time.sleep(self.sample_time - elapsed)
        
        # Stop robot
        self.rtde_control.speedStop()
        print("Control loop finished")
    
    def start_control(self):
        """Start the control loop in a separate thread"""
        if not self.target_poses:
            print("No target poses defined!")
            return
        
        self.control_active = True
        self.current_pose_index = 0
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
    
    def stop_control(self):
        """Stop the control loop"""
        self.control_active = False
        self.rtde_control.speedStop()
        if self.control_thread:
            self.control_thread.join()
    
    def shutdown(self):
        """Shutdown the controller"""
        self.stop_control()
        self.rtde_control.stopScript()

# Example usage
def main():
    # Initialize controller


    controller = UR3eTCPController()
    
    # Move to initial position
    initial_joint_q = [math.radians(45), math.radians(-135), 0.0, 
                      math.radians(-45), math.radians(90), math.radians(45)]
    controller.rtde_control.moveJ(initial_joint_q)
    
    time.sleep(2)
    
    # Define pick-and-place trajectory (similar to paper's Table 2)
    # Pose 1: Hover above pickup
    controller.add_target_pose([0.8, 0.2, 0.5], orientation_rpy=[-1, 1.1, 0])
    
    # Pose 2: Pick up position
    # controller.add_target_pose([0.4, 0.0, 0.2], orientation_rpy=[0, math.pi, 0])
    
    # # Pose 3: Lift up
    # controller.add_target_pose([0.4, 0.0, 0.4], orientation_rpy=[0, math.pi, 0])
    
    # # Pose 4: Move to place position
    # controller.add_target_pose([0.0, 0.4, 0.4], orientation_rpy=[0, math.pi, 0])
    
    # # Pose 5: Place down
    # controller.add_target_pose([0.0, 0.4, 0.2], orientation_rpy=[0, math.pi, 0])
    
    # # Pose 6: Return to home
    # controller.add_target_pose([0.3, 0.0, 0.4], orientation_rpy=[0, math.pi, 0])
    
    try:
        print("Starting TCP control...")
        controller.start_control()
        
        # Wait for completion or user interrupt
        while controller.control_active:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        controller.shutdown()
        print("Controller shutdown complete")

if __name__ == "__main__":
    main()