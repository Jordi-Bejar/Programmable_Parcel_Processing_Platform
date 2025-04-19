import numpy as np
import time
from scipy.interpolate import CubicSpline
import utils 

class Robot:
    def __init__(self):
        self.dof = 5
        self.l_0 = 0.089
        self.l_1 = 0.254
        self.l_2 = 0.33
        self.l_3 = 0
        self.l_4 = 0

    def forward_kinematics(self, thetas):
        dh_parameters = self._get_dh_parameters(thetas)
        frames = np.zeros((4, 4, len(dh_parameters)+1))
        frames[:, :, 0] = np.eye(4)
        H = np.eye(4)

        for i in range(self.dof):
            a, alpha, d, theta = dh_parameters[i,:]

            Rot_x = np.array([
                [1,0,0,0],
                [0,np.cos(alpha),-np.sin(alpha),0],
                [0,np.sin(alpha),np.cos(alpha),0],
                [0,0,0,1]
            ])
            Trans_x = np.array([
                [1,0,0,a],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]
            ])
            Trans_z = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,d],
                [0,0,0,1]
            ])
            Rot_z = np.array([
                [np.cos(theta),-np.sin(theta),0,0],
                [np.sin(theta),np.cos(theta),0,0],
                [0,0,1,0],
                [0,0,0,1]
            ])

            H_i = Rot_x @ Trans_x @ Trans_z @ Rot_z

            H = H @ H_i

            frames[:,:,i+1] = H 

        return frames

    def jacobians(self, thetas):
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        jacobians = np.zeros((6, self.dof, self.dof + 1))
        epsilon = 1e-6
        
        for i in range(self.dof):
            delta = np.zeros(len(thetas))
            delta[i] = epsilon 

            thetas_plus = thetas + delta 
            thetas_minus = thetas - delta 

            frames_plus = self.forward_kinematics(thetas_plus)
            frames_minus = self.forward_kinematics(thetas_minus)

            for j in range(self.dof + 1):
                position_plus = frames_plus[0:3, 3, j]
                position_minus = frames_minus[0:3, 3, j]
                delta_position = (position_plus - position_minus) / (2 * epsilon)

                rotation_plus = frames_plus[0:3, 0:3, j]
                rotation_minus = frames_minus[0:3, 0:3, j]
                diff_R = rotation_plus @ rotation_minus.T

                trace = np.trace(diff_R)

                cos_theta = (trace - 1)/2 
                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                theta = np.arccos(cos_theta) 

                if abs(theta) < 1e-7:
                    delta_rotation = np.zeros(3)
                else:
                    axis_denominator = 2 * np.sin(theta)
                    if abs(axis_denominator) < 1e-7:
                        delta_rotation = np.zeros(3)
                    else:
                        axis = (1 / axis_denominator) * np.array([
                            diff_R[2, 1] - diff_R[1, 2],
                            diff_R[0, 2] - diff_R[2, 0],
                            diff_R[1, 0] - diff_R[0, 1]
                        ])
                        axis_norm = np.linalg.norm(axis)
                        if axis_norm < 1e-6:
                            delta_rotation = np.zeros(3)
                        else:
                            axis = axis / axis_norm
                            delta_rotation = theta * axis / (2*epsilon)    

                jacobians[0:3, i, j] = delta_position
                jacobians[3:6, i, j] = delta_rotation

        return jacobians

    def _inverse_kinematics(self, target_pose, seed_joints):
    
        max_iterations = 20
        tolerance = 1
        joint_lower_limits = np.radians([0, 0, 0, 0, 0])
        joint_upper_limits = np.radians([90, 90, 90, 90, 90])
        learning_rate = 0.1

        joint_angles = seed_joints.copy()

        for step in range(max_iterations):
            all_frames = self.forward_kinematics(joint_angles)
            current_end_effector = all_frames[:, :, -1]
            position_delta = target_pose[:3, 3] - current_end_effector[:3, 3]
            desired_rotation = target_pose[:3, :3]
            current_rotation = current_end_effector[:3, :3]
            rotation_difference = desired_rotation @ current_rotation.T
            trace_value = np.trace(rotation_difference)
            cos_theta = (trace_value - 1) / 2.0
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            angular_error = np.arccos(cos_theta)

            if np.isclose(angular_error, 0):
                orientation_delta = np.zeros(3)
            else:
                orientation_delta = (angular_error / (2 * np.sin(angular_error))) * np.array([
                    rotation_difference[2, 1] - rotation_difference[1, 2],
                    rotation_difference[0, 2] - rotation_difference[2, 0],
                    rotation_difference[1, 0] - rotation_difference[0, 1]
                ])

            total_error = np.concatenate((position_delta, orientation_delta))
            error_norm = np.linalg.norm(total_error)

            if error_norm < tolerance:
                return joint_angles

            jacobian_matrices = self.jacobians(joint_angles)
            jacobian = jacobian_matrices[:, :, -1]
            jacobian_pinv = np.linalg.pinv(jacobian, rcond=1e-6)

            delta_angles = learning_rate * jacobian_pinv @ total_error
            joint_angles += delta_angles

            joint_angles = np.clip(joint_angles, 0.95*joint_lower_limits, 0.95*joint_upper_limits)

        return None
    
    def _get_dh_parameters(self, thetas):
        dh_matrix = np.array([
            [0, np.pi/2, self.l_1, thetas[0]], 
            [self.l_2, 0, 0, thetas[1]], 
            [self.l_3, 0, 0, thetas[2]], 
            [0, np.pi/2, 0, thetas[3] + np.pi/2],
            [0, 0, self.l_3 + self.l_4, thetas[4]]
        ])
    
        return dh_matrix

    def _get_transform(self,R,d):
        transform = np.array([
            [R[0,0],R[0,1],R[0,2],d[0]],
            [R[1,0],R[1,1],R[1,2],d[1]],
            [R[2,0],R[2,1],R[2,2],d[2]],
            [0,0,0,1]
        ])
        return transform

class TrajectoryGenerator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.max_vel = 0.1
        self.max_acc = 0.1
    
    def generate_trapezoidal_trajectory(self, q_start, q_end, max_vel, max_acc, duration):

        delta_q = np.subtract(q_end,q_start)
        n_joints = len(q_start)
        total_distance = np.linalg.norm(delta_q)
        t_acc = duration / 2
        a = 4 * delta_q / duration ** 2
        v_max = a * t_acc

        t_samples = np.arange(0, duration + self.dt, self.dt)
        trajectory = np.zeros((len(t_samples), n_joints))
        
        for i in range(n_joints):
            a_i = a[i]
            q0 = q_start[i]
            v_max_i = v_max[i]
            for idx, t in enumerate(t_samples):
                if t <= t_acc:
                    q = q0 + 0.5 * a_i * t ** 2
                else:
                    t_dec = t - t_acc
                    q_half = q0 + 0.5 * a_i * t_acc ** 2
                    q = q_half + v_max_i * t_dec - 0.5 * a_i * t_dec ** 2
                trajectory[idx, i] = q

        return trajectory 

    def generate_straight_line(self, start_point, end_point, current_joint, start_R, end_R=None, duration=None):
        
        kinematics = Robot()
        times = np.arange(0, duration + self.dt, self.dt)
        t_ends = np.array([times[0],times[-1]])
        x0,y0,z0 = start_point
        x1,y1,z1 = end_point
        bc_type = ((1,0),(1,0))

        cs_x = CubicSpline(t_ends,[x0,x1],bc_type=bc_type)
        cs_y = CubicSpline(t_ends,[y0,y1],bc_type=bc_type)
        cs_z = CubicSpline(t_ends,[z0,z1],bc_type=bc_type)

        smooth_x = cs_x(times)
        smooth_y = cs_y(times)
        smooth_z = cs_z(times)
        trajectory_xyz = np.column_stack((smooth_x,smooth_y,smooth_z))
        trajectory = np.zeros((len(times)-1,7))

        if end_R is None:
            R = start_R
        
            current_transform = kinematics._get_transform(R,start_point)
            target_transform = current_transform

            for i in range(len(times)-1):
                target_transform[:3,-1] = trajectory_xyz[i+1]                           
                current_joint = kinematics._inverse_kinematics(target_transform,current_joint)
                trajectory[i] = current_joint

        else:
            start_quaternion = utils._rotation_to_quaternion(start_R)
            end_quaternion = utils._rotation_to_quaternion(end_R)

            trajectory_R = np.zeros((3,3,len(times)+1))
            for i in range(len(times)):
                current_t = times[i]
                current_quaternion = utils._slerp(start_quaternion,end_quaternion,current_t/duration)
                current_R = utils._quaternion_to_rotation(current_quaternion)
                trajectory_R[:,:,i+1] = current_R

            current_transform = kinematics._get_transform(start_R,start_point)
            target_transform = current_transform

            for i in range(len(times)-1):
                target_transform[:3,:3] = trajectory_R[:,:,i+1]
                target_transform[:3,-1] = trajectory_xyz[i+1]                           
                current_joint = kinematics._inverse_kinematics(target_transform,current_joint)
                trajectory[i] = current_joint
                
        print(trajectory)
        return trajectory


    def get
    
    def follow_joint_trajectory(self, trajectory, send_motor_command):
        for joint_angles in trajectory:
            for i, angle in enumerate(np.degrees(joint_angles)):
                send_motor_command(i+1, angle, speed=1.0)
            time.sleep(self.dt)