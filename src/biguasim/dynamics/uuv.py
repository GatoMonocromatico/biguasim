import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.utils import Transform as tfrm
from biguasim.dynamics.utils import BatchedParams as bp


from biguasim.dynamics.control import BatchedPIDController

class HexaCopterFiveDoF(VehicleModel):
    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)   
        self._control_abstraction = control_abstraction
        self._build_params(params)

        self.submerged_mask = torch.zeros(self.batch_size, 1, device=self.device).double()
        
        # a hack to generate a nice buoyancy behavior :)
        self.dumping_hack = torch.ones(self.batch_size, 1).to(device=self.device)

        self.gen_moments = torch.ones(self.batch_size, 3).to(device=self.device)
        self.gen_moments[:, 1] = 0 

        

    def _build_params(self, params : dict):
        length = params['length']   
        width = params['width']      
        self.batched_params.height = params['height']

        rsphere = ((length + width) / 2)/2
        self.batched_params.rsphere = rsphere        
        
        self.batched_params.I = torch.from_numpy(np.array([params['I'] 
                                                           for _ in range(self.batch_size)])).double().to(self.device)
        
        self.batched_params.volume = (4/3) * np.pi * rsphere**3
        self.batched_params.added_mass = self.batched_params.volume * self.batched_params.rho

        self.batched_params.buoyancy = np.linalg.norm(self.batched_params.rho * \
                                                      self.batched_params.volume * self.batched_params.g)

        self.batched_params.gravity = torch.zeros(self.batch_size, 3, device=self.device).double()
        self.batched_params.gravity[:,-1] = self.batched_params.g

        
        Cp = 0.47 * np.pi * rsphere**2 
        Cpx = Cp
        Cpy = Cp
        Cpz = Cp
        Cr = 0.890e-3 * 8.0 * np.pi * rsphere**3
        Crx = Cr
        Cry = Cr
        Crz = Cr

        self.batched_params.Cp = torch.from_numpy(np.array([np.diag([Cpx, Cpy, Cpz]) 
                                                for _ in range(self.batch_size)])).double().to(self.device)
        self.batched_params.Cr = torch.from_numpy(np.array([np.diag([Crx, Cry, Crz]) 
                                                for _ in range(self.batch_size)])).double().to(self.device)
        
        self.batched_params.sqrt = np.sqrt(2) / 2

        #--------------------- Control
        self.batched_params.rotor_pos = [params["rotor_pos"] for _ in range(self.batch_size)]
        self.batched_params.rotor_dir_np = np.array([params['rotor_directions'] for _ in range(self.batch_size)]) #air
        self.batched_params._extract_geometry()

        x,y,z = params["rotor_pos"]['r1']
        self.batched_params.rotor_distance = np.sqrt(x**2 + y**2 + z**2) *.1

        alloc_mats = []
        for i in range(self.batch_size):
            # Approximate BlueROV2 thruster positions [m] (body frame)
            pos = torch.tensor(
                np.stack([self.batched_params.rotor_pos[i][key] for key in self.batched_params.rotor_pos[i]], axis=0),
                dtype=torch.float64
            ).to(self.device)  # shape (num_rotors, 3)
            # Unit thrust directions for each thruster
            # Horizontal thrusters are angled 45° in XY plane
            thrust_dirs = torch.from_numpy(self.batched_params.rotor_dir_np[i]).double().to(self.device)

            # Compute moment arms (r × F)
            torque_arms = torch.cross(pos, thrust_dirs, dim=-1).T  # (3,8)

            # Build full 6x8 allocation matrix
            A = torch.vstack([
                thrust_dirs.T,   # Fx,Fy,Fz rows
                torque_arms,     # Mx,My,Mz rows
                # + optional reaction_torque if you want small spin drag term
            ])

            alloc_mats.append(A)

        # Use pseudo-inverse for generality
        TM_to_f = torch.stack(alloc_mats).to(self.device)
        self.batched_params.TM_to_f = torch.linalg.pinv(TM_to_f)

        # Low-Level Control Gains
        self.batched_params.k_vxy = torch.tensor([params['k_vxy'] 
                                                for _ in range(self.batch_size)]).unsqueeze(-1).to(self.device)
        
        self.batched_params.k_vz = torch.tensor([params['k_vz'] 
                                                for _ in range(self.batch_size)]).unsqueeze(-1).to(self.device)
        
        self.pid_roll = BatchedPIDController(params['kp_roll'], params['ki_roll'], params['kd_roll'], self.batch_size, self.device)
        
        self.batched_params.kp_yaw = torch.from_numpy(np.array([params['kp_yaw'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kd_yaw = torch.from_numpy(np.array([params['kd_yaw'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        
        self.batched_params.kp_pos = torch.from_numpy(np.array([params['kp_pos'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kd_pos = torch.from_numpy(np.array([params['kd_pos'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        

        
    
    def _compute_external_forces(self, s):
        x = s['x'][self.idxs]
        v = s['v'][self.idxs]
        w = s['w'][self.idxs]

        rho = self.batched_params.rho
        
        Cp = self.batched_params.Cp 
        Cr = self.batched_params.Cr

        self.submerged_mask = torch.ones(self.batch_size, 1, device=self.device).double()
        z = x[:,2].unsqueeze(-1)
        self.submerged_mask[x[:,2] > -0.1] = 0

        factor = 0.5 * self.batched_params.height * self.batched_params.mass
        dumping = self.dumping_hack * (torch.clamp(torch.abs(z), min=0, max= factor) / factor) * self.submerged_mask
        
        #Forces
        W = self.batched_params.weight
        B = (rho * self.batched_params.volume * self.batched_params.gravity) * dumping 
        D = -0.5 * rho * torch.einsum('bij,bj->bi', Cp, v *  torch.abs(v))     
        C = torch.cross(w, self.batched_params.mass * v, dim=1) 
        FtotB = (W + B + D + C) 
        
        #Moments
        M1 = -torch.cross(w, torch.einsum('bij,bj->bi', self.batched_params.I, w), dim=1)
        M2 = -0.5 * rho * torch.einsum('bij,bj->bi', Cr, 
                                                  w *  torch.abs(w) * self.batched_params.rotor_distance)
        MtotB = (M1 + M2)  * self.gen_moments

        return (FtotB, MtotB) 
    

    def _compute_body_wrench(self, rotor_speeds: Tensor):
        """
        5-rotor underwater vehicle:
        - 2 vertical rotors (Z thrust, roll control)
        - 4 at ±45° in XY plane (X, Y thrust, yaw control)
        Supports 5DOF (X, Y, Z, roll, yaw).
        """
        
        self.dumping_hack[(rotor_speeds[:, 0] != 0) & (rotor_speeds[:, 1] != 0)] = 0.5
        TT = self.batched_params.k_eta[self.idxs] * rotor_speeds[self.idxs]**2 * torch.sign(rotor_speeds[self.idxs])
        TT = TT * self.submerged_mask[self.idxs]

        ft_z = TT[:, 0] + TT[:, 1]

        # Rotors 3-6: angled 45° in XY plane
        sqrt = self.batched_params.sqrt

         # --- Corrected X/Y force combinations ---
        # Surge (forward) = +x = (front rotors push forward, rear backward)
        ft_x = ((TT[:, 2] + TT[:, 3]) - (TT[:, 4] + TT[:, 5])) * sqrt

        # Sway (left) = +y = (left rotors push left, right rotors push right)
        ft_y = ((TT[:, 3] + TT[:, 5]) - (TT[:, 2] + TT[:, 4])) * sqrt
        FtotB = torch.stack([ft_x, ft_y, ft_z], dim=1)  # (B,3)

        # ----- Moments (mt) -----
        M = self.batched_params.k_m[self.idxs] * rotor_speeds[self.idxs]**2 * torch.sign(rotor_speeds)# (B,6)
        # Roll from vertical thrusters
        mt_x = -(M[:, 0] - M[:, 1])

        # Pitch negligible for symmetric frame, unless intentionally unbalanced
        mt_y = torch.zeros_like(ft_x)
        # Yaw from counter-torque of horizontal thrusters
        mt_z = (M[:, 2] - M[:, 3] - M[:, 4] + M[:, 5]) * sqrt

        MtotB = torch.stack([mt_x, mt_y, mt_z], dim=1)  # (B,3)

        

        return (FtotB, MtotB)


    def _s_dot_fn(self, s, cmd_rotor_speeds):
        state = self.unpack_state(s, self.idxs, self.batch_size)

        R = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()

        x_dot = state['v'][self.idxs]
        q_dot = tfrm.quat_dot_torch(state['q'][self.idxs], state['w'][self.idxs])

        F, M= self._compute_external_forces(state)
        
        #compute body wrench
        (FtotB, MtotB) = self._compute_body_wrench(cmd_rotor_speeds)

        Ftot = (R @ FtotB.unsqueeze(-1)).squeeze(-1) + F[self.idxs]

        # ------------------------------------------------------------------
        # Compute accelerations
        # ------------------------------------------------------------------
        v_dot = Ftot[self.idxs] /  (self.batched_params.mass + self.batched_params.added_mass)
        w_dot = torch.linalg.solve(self.batched_params.I[self.idxs], MtotB[self.idxs] + M[self.idxs])

    
        # Pack into vector of derivatives.
        s_dot = torch.zeros((len(self.idxs), 13 + cmd_rotor_speeds.shape[1],), device=self.device)
        s_dot[:, 0:3] = x_dot
        s_dot[:, 3:6] = v_dot
        s_dot[:, 6:10] = q_dot  
        s_dot[:, 10:13] = w_dot.squeeze(-1)
        
        return s_dot
        
    
    def get_cmd_motor_speeds(self, state : dict, control : dict):  #control needs to be a tensor
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]
        
        if self._control_abstraction == 'cmd_vel':
            # --- Extract state ---
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz]
            print('----', goal)

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[self.idxs] - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err  # desired acceleration
            a_des[:,2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)

            # --- Roll stabilization ---            
            self.pid_roll.update_setpoints(w[:, 0] * 10)
            M_roll = self.pid_roll.compute(w[:, 0], self.dt, self.idxs)

            # --- Yaw stabilization ---
            yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
            yaw_des = (yaw + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Combine moments (no pitch for 5-DoF) ---
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 2] = M_yaw

            # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
            TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

            # --- Map to thruster forces ---
            cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

            # --- Convert to motor speeds (bidirectional) ---
            cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            # Safety: remove NaNs / infs
            cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            return cmd_motor_speeds
        
        if self._control_abstraction == 'cmd_vel_yaw':
            # --- Extract state ---
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz, yaw_delta_deg]

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[:, :3] - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err  # desired acceleration
            a_des[:,2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)


            # --- Roll stabilization ---            
            self.pid_roll.update_setpoints(w[:, 0] * 10)
            M_roll = self.pid_roll.compute(w[:, 0], self.dt, self.idxs)
            
            # --- Yaw control ---
            yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des = (yaw + yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Combine moments (no pitch for 5-DoF) ---
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 2] = M_yaw

            # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
            TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

            # --- Map to thruster forces ---
            cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

            # --- Convert to motor speeds (bidirectional) ---
            cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            # Safety: remove NaNs / infs
            cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            return cmd_motor_speeds
        
        if self._control_abstraction == 'cmd_pos_yaw':
            # --- Extract state ---
            x = state['x'][self.idxs]          # linear velocity (B,3)
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz, yaw_delta_deg]

            # --- Position control ---
            pos_err = goal[:, :3] - x
            v_des = self.batched_params.kp_pos[self.idxs] * pos_err  # proportional position control
            v_des[:, :2] = torch.clamp(v_des[:, :2], -1.5, 1.5) # Max body velocity
            v_des[:, 2] = torch.clamp(v_des[:, 2], -.15, .15) # Max body velocity

            # --- Velocity control (P controller on velocity error) ---
            v_err = v_des - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err  # desired acceleration
            a_des[:,2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]   # desired force in N (B,3)
            
            # --- Roll stabilization---            
            self.pid_roll.update_setpoints(w[:, 0] * 10)
            M_roll = self.pid_roll.compute(w[:, 0], self.dt, self.idxs)
            
            # --- Yaw control ---
            yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des = (yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Apply yaw-aligned rotation to F_des ---
            R_wb = roma.unitquat_to_rotmat(q).transpose(1,2)   # world->body
            F_des = torch.bmm(R_wb, F_des.unsqueeze(-1)).squeeze(-1)

            # --- Combine moments (no pitch for 5-DoF) ---
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 2] = M_yaw

            # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
            TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

            # --- Map to thruster forces ---
            cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

            # --- Convert to motor speeds (bidirectional) ---
            cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            # Safety: remove NaNs / infs
            cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            return cmd_motor_speeds
            
class OctaCopterSixDoF(HexaCopterFiveDoF):
    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)   
        self._build_params(params)


        self.gen_moments = torch.ones(self.batch_size, 3).to(device=self.device)

        self.pid_roll = BatchedPIDController(10, .1, .01, self.batch_size, self.device)
        

    def _build_params(self, params : dict):
        super()._build_params(params)
        self.pid_pitch = BatchedPIDController(params['kp_pitch'], params['ki_pitch'], params['kd_pitch'], self.batch_size, self.device)


    def _compute_body_wrench(self, rotor_speeds: Tensor):
        """
        6-rotor underwater vehicle:
        - 2 vertical rotors (Z thrust, roll control)
        - 4 at ±45° in XY plane (X, Y thrust, yaw control)
        Supports 5DOF (X, Y, Z, roll, yaw).
        """

        normal_rotors = rotor_speeds[:, :4]    
        vec_rotors = rotor_speeds[:, 4:]  

        #Check minimal action
        self.dumping_hack[(normal_rotors[:, 0] != 0) & (normal_rotors[:, 1] != 0) & \
                          (normal_rotors[:, 2] != 0) & (normal_rotors[:, 3] != 0)] = 0.5
        
        #Normal rotors thrusts
        T = torch.zeros(self.batch_size, 3, 4, device=self.device).double()
        T[..., -1, :] = (self.batched_params.k_eta[self.idxs] * normal_rotors[self.idxs]**2 
                          * torch.sign(normal_rotors) * self.submerged_mask[self.idxs])
        
        NT = T[:, -1]
        
        VT = (self.batched_params.k_eta[self.idxs] * vec_rotors[self.idxs]**2 
            * torch.sign(vec_rotors[self.idxs])) * self.submerged_mask[self.idxs]
        

        # Rotors 3-6: angled 45° in XY plane
        sqrt = self.batched_params.sqrt

        #Forces
        # Surge (forward) = +x = (front rotors push forward, rear backward)
        ft_x = ((VT[:, 0] + VT[:, 1]) - (VT[:, 2] + VT[:, 3])) * sqrt
        # Sway (left) = +y = (left rotors push left, right rotors push right)
        ft_y = ((VT[:, 1] + VT[:, 3]) - (VT[:, 0] + VT[:, 2])) * sqrt

        ft_z = NT[:, 0] + NT[:, 1] + NT[:, 2] + NT[:, 3]

        FtotB = torch.stack([ft_x, ft_y, ft_z], dim=1)  # (B,3)

        # ----- Moments (mt) -----
        VM = self.batched_params.k_m[self.idxs] * vec_rotors[self.idxs]**2 * torch.sign(vec_rotors[self.idxs])# (B,4)        
        
        NM = self.batched_params.k_m[self.idxs] * normal_rotors[self.idxs]**2 * torch.sign(normal_rotors)# (B,6)
        # Roll from vertical thrusters
        mt_x = -(NM[:, 0] + NM[:, 2]) + (NM[:, 1] + NM[:, 3])
        # Pitch negligible for symmetric frame, unless intentionally unbalanced
        mt_y = (NM[:, 0] + NM[:, 1]) - (NM[:, 2] + NM[:, 3])
        # Yaw from counter-torque of horizontal thrusters
        mt_z = (VM[:, 0] - VM[:, 1] - VM[:, 2] + VM[:, 3]) * sqrt

        MtotB = torch.stack([mt_x, mt_y, mt_z], dim=1)  # (B,3)

        return (FtotB, MtotB)
    

    def get_cmd_motor_speeds(self, state : dict, control : dict):  #control needs to be a tensor
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]
        
        if self._control_abstraction == 'cmd_vel':
            # --- Extract state ---
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz]
            print('----', goal)

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[self.idxs] - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err  # desired acceleration
            a_des[:,2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]   # desired force in N (B,3)


            # --- Roll and Pitch stabilization ---
            self.pid_roll.update_setpoints(w[:, 0] * 10)
            self.pid_pitch.update_setpoints(w[:, 1] * 10)

            M_roll = self.pid_roll.compute(w[:, 0], self.dt, self.idxs)
            M_pitch = self.pid_pitch.compute(w[:, 1], self.dt, self.idxs)    


            # --- Yaw stabilization ---
            yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
            yaw_des = (yaw + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi
            
            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Combine moments ---
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 1] = M_pitch
            M_des[:, 2] = M_yaw

            # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
            TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

            # --- Apply yaw-aligned rotation to F_des ---
            R_wb = roma.unitquat_to_rotmat(q).transpose(1,2)   # world->body
            F_des = torch.bmm(R_wb, F_des.unsqueeze(-1)).squeeze(-1)

            # --- Map to thruster forces ---
            cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

            # --- Convert to motor speeds (bidirectional) ---
            cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            # Safety: remove NaNs / infs
            cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            return cmd_motor_speeds
        
        if self._control_abstraction == 'cmd_vel_yaw':
            # --- Extract state ---
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz, yaw_delta_deg]

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[:, :3] - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err  # desired acceleration
            a_des[:,2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]   # desired force in N (B,3)


            # --- Roll and Pitch stabilization ---
            rv = roma.unitquat_to_euler('xyz', q)  # returns (B,3)            
            self.pid_roll.update_setpoints(w[:, 0] * 10)
            self.pid_pitch.update_setpoints(w[:, 1] * 10)

            M_roll = self.pid_roll.compute(w[:, 0], self.dt, self.idxs)
            M_pitch = self.pid_pitch.compute(w[:, 1], self.dt, self.idxs)    
                       
            # --- Yaw control ---
            yaw = rv[:, 2]  # current yaw (ψ)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des = (yaw + yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi
            

            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Combine moments ---
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 1] = M_pitch
            M_des[:, 2] = M_yaw
            
            # --- Combine into total wrench (Fx, Fy, Fz, Mx, My, Mz) ---
            TM = torch.cat([F_des, M_des], dim=-1)  # (B, 6)

            # --- Map to thruster forces ---
            cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

            # --- Convert to motor speeds (bidirectional) ---
            cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            # Safety: remove NaNs / infs
            cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            return cmd_motor_speeds
        
        
        
        if self._control_abstraction == 'cmd_pos_yaw':
            # --- Extract state ---
            x = state['x'][self.idxs]          # linear velocity (B,3)
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz, yaw_delta_deg]

            # --- Position control (outer loop) ---
            pos_err = goal[:, :3] - x
            v_des = self.batched_params.kp_pos[self.idxs] * pos_err  # proportional position control
            v_des[:, :2] = torch.clamp(v_des[:, :2], -1.5, 1.5) # Max body velocity
            v_des[:, 2] = torch.clamp(v_des[:, 2], -.15, .15) # Max body velocity


            # --- Velocity control (P controller on velocity error) ---
            v_err = v_des - v
            a_des = self.batched_params.k_vxy[self.idxs] * v_err  # desired acceleration
            a_des[:,2] *= self.batched_params.k_vz.squeeze(-1)
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)


            # --- Roll and Pitch stabilization ---
            rv = roma.unitquat_to_euler('xyz', q)  # returns (B,3)
            self.pid_roll.update_setpoints(w[:, 0] * 10)
            self.pid_pitch.update_setpoints(w[:, 1] * 10)

            M_roll = self.pid_roll.compute(w[:, 0], self.dt, self.idxs)
            M_pitch = self.pid_pitch.compute(w[:, 1], self.dt, self.idxs)            
           
            # --- Yaw control ---
            yaw = rv[:, 2]  # current yaw (ψ)
            yaw_delta = torch.deg2rad(goal[:, 3].double())
            yaw_des = (yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Combine moments ---
            M_des = torch.zeros_like(F_des)
            M_des[:, 0] = M_roll
            M_des[:, 1] = M_pitch
            M_des[:, 2] = M_yaw


            # --- Apply yaw-aligned rotation to F_des ---
            R_wb = roma.unitquat_to_rotmat(q).transpose(1,2)   # world->body
            F_des = torch.bmm(R_wb, F_des.unsqueeze(-1)).squeeze(-1)

            # --- Combine into total wrench (Fx, Fy, Fz, Mx, Mz) ---
            TM = torch.cat([F_des, M_des], dim=-1)  # (B, 5)

            # --- Map to thruster forces ---
            cmd_thruster_forces = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(-1)).squeeze(-1)

            # --- Convert to motor speeds (bidirectional) ---
            cmd_motor_speeds = cmd_thruster_forces / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            # Safety: remove NaNs / infs
            cmd_motor_speeds = torch.nan_to_num(cmd_motor_speeds, nan=0.0, posinf=0.0, neginf=0.0)

            return cmd_motor_speeds
    

class TorpedoAUV(VehicleModel):
    
    @staticmethod
    def ssa(angle : float):
        """Smallest signed angle [-pi, pi)"""
        return (angle + torch.pi) % (2 * torch.pi) - torch.pi
    
    @staticmethod
    def integralSMC(
        e_int: torch.Tensor,
        e_x: torch.Tensor,
        e_v: torch.Tensor,
        x_d,
        v_d,
        a_d,
        T_nomoto: torch.Tensor,
        K_nomoto: torch.Tensor,
        wn_d: torch.Tensor,
        zeta_d: torch.Tensor,
        K_d: torch.Tensor,
        K_sigma: torch.Tensor,
        lam: torch.Tensor,
        phi_b: torch.Tensor,
        r,
        v_max: torch.Tensor,
        sampleTime: float
    ):
        """
        Integral Sliding Mode Controller (tensor-safe, batch-ready)
        """

        def refModel3(x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime):
            """
            3rd-order reference model (tensor-safe)
            """

            j_d = (
                wn_d**3 * (r - x_d)
                - 3 * wn_d**2 * zeta_d * v_d
                - 3 * wn_d * zeta_d * a_d
            )

            x_d = x_d + sampleTime * v_d
            v_d = v_d + sampleTime * a_d
            a_d = a_d + sampleTime * j_d

            # Tensor-safe saturation
            v_d = torch.clamp(v_d, min=-v_max, max=v_max)

            return x_d, v_d, a_d

        device = e_x.device
        dtype  = e_x.dtype

        # --- force tensor states ---
        x_d = torch.as_tensor(x_d, device=device, dtype=dtype)
        v_d = torch.as_tensor(v_d, device=device, dtype=dtype)
        a_d = torch.as_tensor(a_d, device=device, dtype=dtype)
        r   = torch.as_tensor(r,   device=device, dtype=dtype)

        # Sliding surface
        v_r_dot = a_d - 2 * lam * e_v - lam**2 * e_x
        v_r     = v_d - 2 * lam * e_x - lam**2 * e_int
        sigma   = e_v + 2 * lam * e_x + lam**2 * e_int

        # Control law (fully tensorized)
        abs_ratio = torch.abs(sigma / phi_b)

        delta = torch.where(
            abs_ratio > 1.0,
            (T_nomoto * v_r_dot + v_r - K_d * sigma - K_sigma * torch.sign(sigma)) / K_nomoto,
            (T_nomoto * v_r_dot + v_r - K_d * sigma - K_sigma * (sigma / phi_b)) / K_nomoto
        )

        # Integral update
        e_int = e_int + sampleTime * e_x

        # Reference model
        x_d, v_d, a_d = refModel3(
            x_d, v_d, a_d, r, wn_d, zeta_d, v_max, sampleTime
        )

        return delta, e_int, x_d, v_d, a_d

    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)
        self._build_params(params)

        
        self.u_actual = None
    
    def _build_params(self, params : dict):
        self.batched_params.D2R = np.pi / 180
        self.batched_params.length = params['length']
        self.batched_params.diam = params['diam']

        self.batched_params.r_bg = torch.from_numpy(np.array([params['r_bg'] 
                                                             for _ in range(self.batch_size)])).to(self.device)
        self.batched_params.r_bb = torch.from_numpy(np.array([params['r_bb'] 
                                                             for _ in range(self.batch_size)])).to(self.device)

        

        self.batched_params.r44 = params['r44']
        self.batched_params.Cd = params['Cd']
        self.batched_params.T_surge = params['T_surge']
        self.batched_params.T_sway = params['T_sway']
        self.batched_params.zeta_roll = params['zeta_roll']
        self.batched_params.zeta_pitch = params['zeta_pitch']
        self.batched_params.T_yaw = params['T_yaw']
        self.batched_params.K_nomoto = params['K_nomoto']

        self.batched_params.S_fin = params['fin_area']
        self.batched_params.fin_center = params['fin_center']
        self.batched_params.deltaMax_fin = params['deltaMax_fin_deg']
        self.batched_params.nMax = params['nMax']
        self.batched_params.T_delta = params['T_delta']
        self.batched_params.T_n = params['T_n']
        self.batched_params.CL_delta_r = params['CL_delta_r']
        self.batched_params.CL_delta_s = params['CL_delta_s']

        self.batched_params.wn_d_z = params['wn_d_z']
        self.batched_params.Kp_z = params['Kp_z']
        self.batched_params.T_z = params['T_z']
        self.batched_params.Kp_theta = params['Kp_theta']
        self.batched_params.Kd_theta = params['Kd_theta']
        self.batched_params.Ki_theta = params['Ki_theta']
        self.batched_params.K_w = params['K_w']
        self.batched_params.theta_max = params['theta_max_deg'] * self.batched_params.D2R

        self.batched_params.wn_d = params['wn_d']
        self.batched_params.zeta_d = params['zeta_d']
        self.batched_params.r_max = params['r_max']
        self.batched_params.lam = params['lam']
        self.batched_params.phi_b = params['phi_b']
        self.batched_params.K_d = params['K_d']
        self.batched_params.K_sigma = params['K_sigma']

        self.batched_params.Kp_surge = params['kp_surge']
        self.batched_params.Kd_surge = params['kd_surge']
        self.batched_params.Ki_surge = params['ki_surge']

        self.batched_params.r_z = 0
        self.batched_params.r_psi = 0
        self.batched_params.r_rpm = 0
        self.batched_params.r_surge = 0
        self.batched_params.V_c = 0 
        self.batched_params.beta_c = 0
        self.batched_params.nu = torch.zeros(self.batch_size, 6, device=self.device).double()

        self.u_int = 0   # surge error integral state
        self._last_error = 0

        self.z_int = 0         # heave position integral state
        self.z_d = 0           # desired position, LP filter initial state
        self.theta_int = 0     # pitch angle integral state
        self.psi_d = 0   # desired heading from control loop
        self.r_d = 0     # desired yaw rate from control loop
        self.a_d = 0
        self.e_psi_int = 0   # yaw angle error integral state
        self.prev_pitch = 0
        self.prev_yaw = 0


        # ---------------------------------------------------------------------------------------
        self.batched_params.S = 0.7 * self.batched_params.length * self.batched_params.diam
        self.batched_params.a = self.batched_params.length * 0.5
        b = self.batched_params.diam * 0.5

        self.batched_params.CD_0 = self.batched_params.Cd * np.pi * b**2 / self.batched_params.S

        m = self.batched_params.mass
        Ix = (2/5) * m * b**2                       # moment of inertia
        Iy = (1/5) * m * (self.batched_params.a**2 + b**2)
        Iz = Iy

        MRB_CG = torch.from_numpy(np.array([np.diag([ m, m, m, 
                                                     Ix, Iy, Iz ]) for _ in range(self.batch_size)])).to(self.device)   # MRB expressed in the CG     

        H_rg = bp.Hmtrx(self.batched_params.r_bg)
        self.batched_params.MRB = H_rg.mT @ MRB_CG @ H_rg

        self.batched_params.W = self.batched_params.mass * self.batched_params.g
        self.batched_params.B = self.batched_params.W

        MA_44 = self.batched_params.r44 * Ix 

        e = np.sqrt(1.0 - (b / self.batched_params.a) ** 2)
        log_term = np.log((1.0 + e) / (1.0 - e))

        alpha_0 = (
            2.0 * (1.0 - e**2) / e**3
        ) * (0.5 * log_term - e)                        # (B,)

        beta_0 = (
            1.0 / e**2
            - (1.0 - e**2) / (2.0 * e**3) * log_term
        )                                               # (B,)

        k1 = alpha_0 / (2.0 - alpha_0)                  # (B,)
        k2 = beta_0 / (2.0 - beta_0)                    # (B,)

        k_prime = (
            e**4 * (beta_0 - alpha_0)
            / (
                (2.0 - e**2)
                * (2.0 * e**2 - (2.0 - e**2) * (beta_0 - alpha_0))
            )
        ) 

        self.batched_params.MA = torch.from_numpy(np.array([np.diag([ m*k1, m*k2, m*k2, MA_44, k_prime*Iy, k_prime*Iy ]) 
                                                            for _ in range(self.batch_size)])).to(self.device)
          
        # Mass matrix including added mass
        self.batched_params.M = self.batched_params.MRB + self.batched_params.MA
        self.batched_params.Minv = torch.linalg.inv(self.batched_params.M)

        dz = self.batched_params.r_bg[:, 2] - self.batched_params.r_bb[:, 2]          # (B,)
        self.w_roll = torch.sqrt(
            self.batched_params.W * dz / self.batched_params.M[:, 3, 3]
        )   

        self.w_pitch = torch.sqrt(
            self.batched_params.W * dz / self.batched_params.M[:, 4, 4]
        ) 

        self.batched_params.T_heave = self.batched_params.T_sway                      # (B,)
        self.batched_params.T_nomoto = self.batched_params.T_yaw                      # (B,)

        # Tail rudder parameters
        self.A_r = self.batched_params.S_fin
        self.x_r = -self.batched_params.a

        # Stern-plane parameters
        self.A_s = self.batched_params.S_fin
        self.x_s = -self.batched_params.a

        self.surge_control = False

    
    def _forceLiftDrag(self, alpha, U_r) -> torch.Tensor:
        """
        Hydrodynamic lift and drag of a fin/wing (batch version)
        
        Inputs:
            b, S, CD_0, alpha, U_r: tensors of shape (B,) or (B,1)
        Returns:
            tau_liftdrag: tensor of shape (B,6)
        """

        # Compute lift and drag coefficients
        e = 0.7
        AR = self.batched_params.diam**2 / self.batched_params.S
        CL_alpha = np.pi * AR / (1 + np.sqrt(1 + (AR/2)**2))
        CL = CL_alpha * alpha
        CD = self.batched_params.CD_0 + CL**2 / (torch.pi * e * AR)

        # Lift and drag forces
        F_drag = 0.5 * self.batched_params.rho * self.batched_params.S * CD * U_r**2
        F_lift = 0.5 * self.batched_params.rho * self.batched_params.S * CL * U_r**2

        # Compute batch tau_liftdrag
        tau_liftdrag = torch.stack([
            torch.cos(alpha) * (-F_drag) - torch.sin(alpha) * (-F_lift),
            torch.zeros_like(alpha),
            torch.sin(alpha) * (-F_drag) + torch.cos(alpha) * (-F_lift),
            torch.zeros_like(alpha),
            torch.zeros_like(alpha),
            torch.zeros_like(alpha)
        ])  # shape: (B,6)

        return tau_liftdrag
    
    
    def _gvect(self) -> Tensor:
        """
        g = gvect(W,B,theta,phi,r_bg,r_bb) computes the 6x1 vector of restoring 
        forces about an arbitrarily point CO for a submerged body. 
        
        Inputs:
            W, B: weight and buoyancy (kg)
            phi,theta: roll and pitch angles (rad)
            r_bg = [x_g y_g z_g]: location of the CG with respect to the CO (m)
            r_bb = [x_b y_b z_b]: location of the CB with respect to th CO (m)
            
        Returns:
            g: 6x1 vector of restoring forces about CO
        """
        batch_size = self.batch_size
        device = self.device
            
        w = self.batched_params.W
        b = self.batched_params.B

        r_bg = self.batched_params.r_bg
        r_bb = self.batched_params.r_bb

        theta = self.eta[:, 4]
        phi = self.eta[:, 3]
        

        sth  = torch.sin(theta).unsqueeze(-1).to(device)
        cth  = torch.cos(theta).unsqueeze(1).to(device)
        sphi = torch.sin(phi).unsqueeze(1).to(device)
        cphi = torch.cos(phi).unsqueeze(1).to(device)

        g = torch.zeros((batch_size, 6, 1), device=device).double()
        
        g[:, 0] = (w-b) * sth
        g[:, 1] = -(w-b) * cth * sphi
        g[:, 2] = -(w-b) * cth * cphi
        g[:, 3] = -(r_bg[:,1:2]*w-r_bb[:,1:2]*b) * cth * cphi + (r_bg[:,2:3]*\
                                                w-r_bb[:,2:3]*b) * cth * sphi
        g[:, 4] = (r_bg[:,2:3]*w-r_bb[:,2:3]*b) * sth  + (r_bg[:,0:1]*w\
                                            -r_bb[:,0:1]*b) * cth * cphi
        g[:, 5] = -(r_bg[:,0:1]*w-r_bb[:,0:1]*b) * cth * sphi - (r_bg[:,1:2]*\
                                                        w-r_bb[:,1:2]*b) * sth
        return g.transpose(1, 2).squeeze(1)

    def _compute_body_wrench(self, u_actual, nu_r):
        """
        GPU-friendly actuator dynamics.
        u_actual: (B, 5)
        nu_r: (B, 6)
        Returns: forces (B,3), moments (B,3)
        """

        delta_rt = u_actual[:, 0]
        delta_rb = u_actual[:, 1]
        delta_sl = u_actual[:, 2]
        delta_sr = u_actual[:, 3]

        # Horizontal and vertical relative speeds
        U_rh = torch.sqrt(nu_r[:, 0]**2 + nu_r[:, 1]**2)
        U_rv = torch.sqrt(nu_r[:, 0]**2 + nu_r[:, 2]**2)

        # Rudder and stern-plane drag (always negative)
        X_r = -0.5 * self.batched_params.rho * U_rh**2 * self.A_r * self.batched_params.CL_delta_r * delta_rt**2
        X_r += -0.5 * self.batched_params.rho * U_rh**2 * self.A_r * self.batched_params.CL_delta_r * delta_rb**2
        X_s = -0.5 * self.batched_params.rho * U_rv**2 * self.A_s * self.batched_params.CL_delta_s * delta_sl**2
        X_s += -0.5 * self.batched_params.rho * U_rv**2 * self.A_s * self.batched_params.CL_delta_s * delta_sr**2

        # Rudder sway force
        Y_r = -0.5 * self.batched_params.rho * U_rh**2 * self.A_r * self.batched_params.CL_delta_r * delta_rt
        Y_r += -0.5 * self.batched_params.rho * U_rh**2 * self.A_r * self.batched_params.CL_delta_r * delta_rb

        # Stern-plane heave force
        Z_s = 0.5 * self.batched_params.rho * U_rv**2 * self.A_s * self.batched_params.CL_delta_s * delta_sl
        Z_s += 0.5 * self.batched_params.rho * U_rv**2 * self.A_s * self.batched_params.CL_delta_s * delta_sr

        forces = torch.stack([X_r + X_s, Y_r, Z_s], dim=1)
        moments = torch.stack([torch.zeros_like(X_r), -self.x_s * Z_s, self.x_r * Y_r], dim=1)

        return forces, moments
    
    def _compute_external_forces(self, alpha, U_r, nu_r, tau):
        # --------------------------------------------------
        # Coriolis and centripetal matrices
        # --------------------------------------------------
        CRB = self.batched_params.m2c(self.batched_params.MRB, nu_r)                         # (B,6,6)
        CA = self.batched_params.m2c(self.batched_params.MA, nu_r)

        # Zero destabilizing CA terms (batched indexing)
        CA[:, 4, 0] = 0.0
        CA[:, 0, 4] = 0.0
        CA[:, 2, 4] = 0.0
        CA[:, 5, 0] = 0.0
        CA[:, 5, 1] = 0.0
        CA[:, 1, 5] = 0.0

        C = CRB + CA                                     # (B,6,6)

        # --------------------------------------------------
        # Linear damping matrix
        # --------------------------------------------------
        D = torch.diag_embed(
            torch.stack(
                [
                    self.batched_params.M[:, 0, 0] / self.batched_params.T_surge,
                    self.batched_params.M[:, 1, 1] / self.batched_params.T_sway,
                    self.batched_params.M[:, 2, 2] / self.batched_params.T_heave,
                    self.batched_params.M[:, 3, 3] * 2.0 * self.batched_params.zeta_roll  * self.w_roll,
                    self.batched_params.M[:, 4, 4] * 2.0 * self.batched_params.zeta_pitch * self.w_pitch,
                    self.batched_params.M[:, 5, 5] / self.batched_params.T_yaw,
                ],
                dim=1,
            )
        )                                                 # (B,6,6)


        exp_term = torch.exp(-3.0 * U_r)
        D[:, 0, 0] = D[:, 0, 0] * exp_term
        D[:, 1, 1] = D[:, 1, 1] * exp_term

        # --------------------------------------------------
        # Hydrodynamic forces
        # --------------------------------------------------
        tau_liftdrag = self._forceLiftDrag(alpha, U_r)    

        tau_crossflow = bp.cross_flow_drag(self.batched_params.rho, self.batched_params.length, self.batched_params.diam, self.batched_params.diam, nu_r) 

        # --------------------------------------------------
        # Restoring forces
        # --------------------------------------------------
        g = self._gvect()                                            # (B,6)
 
        # --------------------------------------------------
        # Equations of motion
        # --------------------------------------------------
        tau_sum = (
            tau 
            + tau_liftdrag.T 
            + tau_crossflow.squeeze(-1) 
            - torch.einsum('bij,bj->bi', C+D, nu_r)
        ).squeeze(-1) - g  
        
        return tau_sum
        # return (C, D, tau_liftdrag, tau_crossflow, g)



    def _s_dot_fn(self, s, cmd_ctrl):
        state = self.unpack_state(s, self.idxs, self.batch_size)        

        # Current velocities
        u_c = self.batched_params.V_c * torch.cos(self.batched_params.beta_c - self.eta[:, 5])  # current surge velocity
        v_c = self.batched_params.V_c * torch.sin(self.batched_params.beta_c - self.eta[:, 5])  # current sway velocity

        
        nu_c = torch.cat([u_c.unsqueeze(1), v_c.unsqueeze(1), torch.zeros(self.batch_size, 4, device=self.device)], dim=1).double() # current velocity 
        Dnu_c = torch.cat([self.nu[:, 5].unsqueeze(1)*v_c, -self.nu[:, 5].unsqueeze(1)*u_c,  torch.zeros(self.batch_size, 4, device=self.device)], dim=1).double() # derivative
        nu_r = self.nu - nu_c                               # relative velocity        
        alpha = torch.arctan2( nu_r[:, 2], nu_r[:, 0] )         # angle of attack 
        U = torch.sqrt(self.nu[:, 0]**2 + self.nu[:, 1]**2 + self.nu[:, 2]**2)  # vehicle speed
        U_r = torch.sqrt(nu_r[:, 0]**2 + nu_r[:, 1]**2 + nu_r[:, 2]**2)  # relative speed

        forces, moments = self._compute_body_wrench(self.u_actual, nu_r)

        n = self.u_actual[:, -1]                              # (B,)

        D_prop = torch.full_like(n, 0.14)
        t_prop = torch.full_like(n, 0.1)

        n_rps = n / 60.0
        Va = 0.944 * U

        Ja_max = torch.full_like(n, 0.6632)

        KT_0 = torch.full_like(n, 0.4566)
        KQ_0 = torch.full_like(n, 0.0700)
        KT_max = torch.full_like(n, 0.1798)
        KQ_max = torch.full_like(n, 0.0312)

        abs_n = torch.abs(n_rps)

        X_fwd = self.batched_params.rho * D_prop**4 * (
            KT_0 * abs_n * n_rps
            + (KT_max - KT_0) / Ja_max * (Va / D_prop) * abs_n
        )

        K_fwd = self.batched_params.rho * D_prop**5 * (
            KQ_0 * abs_n * n_rps
            + (KQ_max - KQ_0) / Ja_max * (Va / D_prop) * abs_n
        )

        X_rev = self.batched_params.rho * D_prop**4 * KT_0 * abs_n * n_rps
        K_rev = self.batched_params.rho * D_prop**5 * KQ_0 * abs_n * n_rps

        forward = n_rps > 0

        X_prop = torch.where(forward, X_fwd, X_rev)      # (B,)
        K_prop = torch.where(forward, K_fwd, K_rev)      # (B,)

        tau = torch.stack(
            [
                (1.0 - t_prop) * X_prop + forces[:, 0],
                forces[:, 1],
                forces[:, 2],
                K_prop / 10.0 + moments[:, 0],
                moments[:, 1],
                moments[:, 2],
            ],
            dim=1,
        )  

        tau_sum = self._compute_external_forces(alpha, U_r, nu_r, tau) 
        nu_dot = Dnu_c + (self.batched_params.Minv @ tau_sum.unsqueeze(-1)).squeeze(-1)
        
        u_actual_dot = torch.empty_like(self.u_actual)
        # Fin dynamics
        u_actual_dot[:-1] = (cmd_ctrl[:-1] - self.u_actual[:-1]) / self.batched_params.T_delta

        # Thruster dynamics
        u_actual_dot[-1] = (cmd_ctrl[-1] - self.u_actual[-1]) / self.batched_params.T_n

        # Euler integration (in-place, GPU-safe)
        self.u_actual += (1/50) * u_actual_dot

        self.u_actual[:-1] = torch.clamp(
            self.u_actual[:-1],
            min=-self.batched_params.deltaMax_fin,
            max=self.batched_params.deltaMax_fin
        )

        self.u_actual[-1] = torch.clamp(
            self.u_actual[-1],
            min=-self.batched_params.nMax,
            max=self.batched_params.nMax
        )
        
        nu_dot = tfrm.convert_NED_to_NWU(state['q'], nu_dot)
        # Pack into vector of derivatives.
        s_dot = torch.zeros((len(self.idxs), 13 + cmd_ctrl.shape[1],), device=self.device)
        s_dot[:, 0:3] = state['v'][self.idxs]
        s_dot[:, 3:6] = nu_dot[:, 0:3]
        s_dot[:, 6:10] = state['q'][self.idxs]
        s_dot[:, 10:13] = nu_dot[:, 3:6]
        
        return s_dot
    
    def get_cmd_motor_speeds(self, state, control):        
        self.eta, self.nu = tfrm.convert_NWU_to_NED(state['x'][self.idxs], state['q'][self.idxs], state['v'][self.idxs], state['w'][self.idxs])

        
        if not isinstance(self.u_actual, Tensor):
                self.u_actual = torch.zeros(self.batch_size, 5, device=self.device).double()

        if self.control_abstraction == 'cmd_rudders_sterns_motor_speed':
            return control['cmd_ctrl'][self.idxs]
        
        if self.control_abstraction == 'cmd_depth_heading_rpm_surge':
            """
            Simultaneously control the heading and depth of the AUV using control laws of PID type.
            Propeller rpm is given as a step command.

            :param array-like eta: State/pose of the vehicle in the world frame. (RPY - Euler angle order zyx in radians)
            :param array-like nu: Velocity of the vehicle in the body frame.
            :param float sampleTime: Time since the last step.

            :returns: The control input u_control.
            """
            
            self.ref_z = control['cmd_ctrl'][:, 0]
            self.ref_psi = control['cmd_ctrl'][:, 1]
            self.ref_n = control['cmd_ctrl'][:, 2]
            self.ref_u = control['cmd_ctrl'][:, 3]
            self.surge_control = True if self.ref_u > 0 else False

            z = self.eta[:, 2]                  # heave position (depth)
            theta = self.eta[:, 4]              # pitch angle (Radians)
            psi = self.eta[:, 5]                # yaw angle   (Radians)
            w = self.nu[:, 2]                   # heave velocity

            q = self.nu[:,4]                   # pitch rate
            r = self.nu[:,5]                   # yaw rate
            

            e_psi = psi - self.psi_d    # yaw angle tracking error
            e_r   = r - self.r_d        # yaw rate tracking error
            z_ref = self.ref_z          # heave position (depth) setpoint
            psi_ref = self.ref_psi * self.batched_params.D2R   # yaw angle setpoint
            
            #If surge command is 0 then control loop should not run 
            if self.ref_n > 0 or self.ref_u > 0:
                

                if self.surge_control:
                    u = self.nu[:, 0]                   # surge velocity
                    # TODO: get nu_dot from linear acceleration IMU - gravity
                    # udot = nu_dot[0]            # surge acceleration

                    setpoint = self.ref_u
                    error = setpoint - u
                    derivative = (error - self._last_error) / (1/50)

                    n = self.batched_params.Kp_surge * error + self.batched_params.Ki_surge * self.u_int + self.batched_params.Kd_surge * derivative

                    self.u_int += (1/50) * (error)

                    n = torch.clamp(n, min=-self.batched_params.nMax, max=self.batched_params.nMax)    #Max out surge controller to the propeller command

                else:
                    n = self.ref_n
                #######################################################################            
                # Depth autopilot (succesive loop closure)
                #######################################################################
                # LP filtered desired depth command 
                self.z_d = z    #On initialization of the autopilot the commanded depth is set to the current depth
                self.z_d  = np.exp( -(1/50) * self.batched_params.wn_d_z ) * self.z_d \
                    + ( 1 - np.exp( -(1/50) * self.batched_params.wn_d_z) ) * z_ref  
                    
                # PI controller    
                theta_d = self.batched_params.Kp_z * ( (z - self.z_d) + (1/self.batched_params.T_z) * self.z_int )

                if abs(theta_d) > self.batched_params.theta_max:
                    theta_d = torch.sign(theta_d) * self.batched_params.theta_max

                delta_s = -self.batched_params.Kp_theta * self.ssa( theta - theta_d ) - self.batched_params.Kd_theta * q \
                    - self.batched_params.Ki_theta * self.theta_int - self.batched_params.K_w * w

                # Euler's integration method (k+1)
                self.z_int     += (1/50) * ( z - self.z_d )
                self.theta_int += (1/50) * self.ssa( theta - theta_d )

                #######################################################################
                # Heading autopilot (SMC controller)
                #######################################################################
                
                wn_d = self.batched_params.wn_d            # reference model natural frequency
                zeta_d = self.batched_params.zeta_d        # reference model relative damping factor


                # Integral SMC with 3rd-order reference model
                [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
                    self.integralSMC( 
                        self.e_psi_int, 
                        e_psi, e_r, 
                        self.psi_d, 
                        self.r_d, 
                        self.a_d, 
                        self.batched_params.T_nomoto, 
                        self.batched_params.K_nomoto, 
                        wn_d, 
                        zeta_d, 
                        self.batched_params.K_d, 
                        self.batched_params.K_sigma, 
                        self.batched_params.lam,
                        self.batched_params.phi_b,
                        psi_ref, 
                        self.batched_params.r_max, 
                        1/50
                        )
                        
                # Euler's integration method (k+1)
                self.e_psi_int += (1/50) * self.ssa( psi - self.psi_d )
                            
                # u_control = torch.tensor([ delta_r, delta_s, n], device=self.device).double()
                u_control = torch.hstack((delta_r, delta_s, n)).unsqueeze(0)

            else:
                u_control = torch.zeros(self.batch_size, 3, device=self.device).double()
        

            delta_rt = delta_rb = u_control[:, 0]
            delta_sl = delta_sr = u_control[:, 1]
            n = u_control[:, 2]

            u_control = torch.stack([delta_rt, delta_rb,delta_sl,delta_sr, n], dim=1)
            
            return u_control
        





















































































