import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.utils import Transform as tfrm
from biguasim.dynamics.utils import BatchedParams as bp








class Catamaran(VehicleModel):
    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)   
        self._control_abstraction = control_abstraction
        self._build_params(params)

        self.submerged_mask = torch.zeros(self.batch_size, 1, device=self.device).double()
        
        # a hack to generate a nice buoyancy behavior :)
        self.dumping_hack = torch.ones(self.batch_size, 1).to(device=self.device)

        self.gen_moments = torch.ones(self.batch_size, 3).to(device=self.device)
        

    def _build_params(self, params : dict):
        length = params['length']   
        width = params['width']      
        height =  params['height']
        self.batched_params.height = height

        vessel = width * length * height     
        self.batched_params.I = torch.from_numpy(np.array([params['I'] 
                                                           for _ in range(self.batch_size)])).double().to(self.device)
        
        #ellipsoid
        self.batched_params.volume = (4/3) * np.pi * (2*width) * length * (height/2)
        self.batched_params.added_mass = self.batched_params.volume * self.batched_params.rho

        self.batched_params.buoyancy = np.linalg.norm(self.batched_params.rho * \
                                                      self.batched_params.volume * self.batched_params.g)

        self.batched_params.gravity = torch.zeros(self.batch_size, 3, device=self.device).double()
        self.batched_params.gravity[:,-1] = self.batched_params.g


        Cd = 0.38 #Hoerner's book param (pg - 60)
        Cpz = (Cd * np.pi * width * length)
        Cpx = (Cd * np.pi * width * height)
        Cpy = (Cd * np.pi * length * height)
        Crx = (Cd * np.pi * width * length)
        Cry = (Cd * np.pi * vessel)
        Crz = Cry


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
        self.batched_params.k_vx = torch.tensor([params['k_vx'] 
                                                for _ in range(self.batch_size)]).unsqueeze(-1).to(self.device)
        
        
        # self.pid_roll = BatchedPIDController(params['kp_roll'], params['ki_roll'], params['kd_roll'], self.batch_size, self.device)
        
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
        self.submerged_mask[(x[:,2] > -(self.batched_params.height/2))] = 0

        factor = .5 * self.batched_params.height * self.batched_params.mass
        dumping = .7 * self.dumping_hack * (torch.clamp(torch.abs(z), min=0, max= factor) / factor) * self.submerged_mask
        
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

                # Surge (forward) = +x = (front rotors push forward, rear backward)
        ft_x = TT[:, 0] + TT[:, 1]
        ft_y = torch.zeros_like(ft_x)
        ft_z = torch.zeros_like(ft_x)
        FtotB = torch.stack([ft_x, ft_y, ft_z], dim=1)  # (B,3)

        # ----- Moments (mt) -----
        M = self.batched_params.k_m[self.idxs] * rotor_speeds[self.idxs]**2 * torch.sign(rotor_speeds)# (B,6)
        # Roll from vertical thrusters
        mt_x = torch.zeros_like(ft_x)

        # Pitch negligible for symmetric frame, unless intentionally unbalanced
        mt_y = torch.zeros_like(ft_x)
        # Yaw from counter-torque of horizontal thrusters
        mt_z = (M[:, 0] - M[:, 1])

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
            goal[:, 1:] = 0

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[self.idxs] - v
            a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
            a_des[:, 1:] = 0
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)

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
            goal[:, 1] = 0
            goal[:, 2] = 0

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[:, :3] - v
            a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
            a_des[:, 1:] = 0
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)

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
            pos_err = torch.zeros(self.batch_size, 3, device=self.device).double()
            pos_err[:, 0] = torch.sqrt((goal[:,0] - x[:,0])**2 + (goal[:,1] - x[:,1])**2)

            yaw_delta = torch.atan2(goal[:, 1] - x[:, 1], goal[:, 0] - x[:, 0])

            v_des = self.batched_params.kp_pos[self.idxs] * pos_err  # proportional position control
            v_des[:, 0] = torch.clamp(v_des[:,0], -2.5, 3) # Max body velocity 

            # --- Velocity control (P controller on velocity error)  ---
            v_err = v_des - v 
            a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)

            # --- Yaw control ---
            yaw = roma.unitquat_to_rotvec(q)[:, 2]  # current yaw (ψ)
            yaw_des = (yaw_delta + np.pi) % (2 * np.pi) - np.pi
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            yaw_rate = w[:, 2]
            yaw_rate_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_rate_des - yaw_rate
            M_yaw = -self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err
            # --- Apply yaw-aligned rotation to F_des ---

            # R_wb = roma.unitquat_to_rotmat(q).transpose(1,2)   # world->body
            # F_des = torch.bmm(R_wb, F_des.unsqueeze(-1)).squeeze(-1)

            # --- Combine moments (no pitch for 5-DoF) ---
            M_des = torch.zeros_like(F_des)
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
        


class Differential(VehicleModel):
    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)   
        self._control_abstraction = control_abstraction
        self._build_params(params)

        self.surface_mask = torch.zeros(self.batch_size, 1, device=self.device).double()

    def _build_params(self, params):
        
        self.batched_params.L = 1.195    # length (m)
        self.batched_params.B = .19   # beam (m)

        self.batched_params.V_c = 0
        self.batched_params.beta_c = 0 * (np.pi / 180 ) #TODO set spawn angle from config file (deg) instead zero.

        self.batched_params.r_bg = torch.from_numpy(np.array([params['r_bg'] 
                                                             for _ in range(self.batch_size)])).to(self.device)
        
        self.batched_params.S_rg = bp.hat_map(self.batched_params.r_bg).permute(2, 0, 1)
        self.batched_params.H_rg = bp.Hmtrx(self.batched_params.r_bg)
        self.batched_params.zeta_d = params['zeta_d']  # desired relative damping ratio

        R44 = 0.2 * self.batched_params.B  # radii of gyration (m)
        R55 = 0.25 * self.batched_params.L
        R66 = 0.25 * self.batched_params.L

        T_sway = params['T_sway']      # time constant in sway (s)
        T_yaw = params['T_yaw']      # time constant in yaw (s)
        Umax = self.batched_params.rotor_speed_max   # max forward speed (m/s)

        # Data for one pontoon
        self.batched_params.B_pont = 0.299  # beam of one pontoon (m)
        y_pont = params['y_pont']      # distance from centerline to waterline centroid (m)
        Cw_pont = params['Cw_pont']      # waterline area coefficient (-)
        Cb_pont = params['Cb_pont']       # block coefficient

        # Inertia dyadic, volume displacement and draft
        nabla = self.batched_params.mass / self.batched_params.rho  # volume
        self.batched_params.T = nabla / (2 * Cb_pont * self.batched_params.B_pont * self.batched_params.L)  # draft
        Ig_CG = self.batched_params.mass * torch.from_numpy(np.array([np.diag(np.array([R44 ** 2, R55 ** 2, R66 ** 2])) 
                                                                      for _ in range(self.batch_size)])).to(self.device)
        self.batched_params.Ig = Ig_CG - self.batched_params.mass * self.batched_params.S_rg @ self.batched_params.S_rg

        # Experimental propeller data including lever arms
        self.batched_params.l1 = -y_pont  # lever arm, left propeller (m)
        self.batched_params.l2 = y_pont  # lever arm, right propeller (m)
        self.batched_params.k_pos = 0.02216 / 2  # Positive Bollard, one propeller
        self.batched_params.k_neg = 0.01289 / 2  # Negative Bollard, one propeller
        self.batched_params.n_max = np.sqrt((0.5 * 24.4 * self.batched_params.g) / self.batched_params.k_pos)  # max. prop. rev.
        self.batched_params.n_min = -np.sqrt((0.5 * 13.6 * self.batched_params.g) / self.batched_params.k_neg) # min. prop. rev.

        # MRB_CG = [ (m+mp) * I3  O3      (Fossen 2021, Chapter 3)
        #               O3       Ig ]
        MRB_CG = torch.zeros((self.batch_size, 6, 6)).double().to(self.device)
        MRB_CG[:, 0:3, 0:3] = self.batched_params.mass * torch.eye(3)
        MRB_CG[:, 3:6, 3:6] = self.batched_params.Ig
        MRB = self.batched_params.H_rg.mT @ MRB_CG @ self.batched_params.H_rg

        # Hydrodynamic added mass (best practice)
        Xudot = -0.1 * torch.from_numpy(np.array([ self.batched_params.mass 
                                                 for _ in range(self.batch_size)])).to(self.device)
        Yvdot = -1.5 * torch.from_numpy(np.array([ self.batched_params.mass 
                                                 for _ in range(self.batch_size)])).to(self.device)
        Zwdot = -1.0 * torch.from_numpy(np.array([ self.batched_params.mass 
                                                 for _ in range(self.batch_size)])).to(self.device)
        Kpdot = -0.2 * self.batched_params.Ig[:, 0, 0]
        Mqdot = -0.8 * self.batched_params.Ig[:, 1, 1]
        Nrdot = -1.7 * self.batched_params.Ig[:, 2, 2]

        self.batched_params.MA = -torch.diag_embed(torch.stack([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot], dim=1))

        # System mass matrix
        self.batched_params.M = MRB + self.batched_params.MA
        self.batched_params.Minv = torch.linalg.inv(self.batched_params.M)

        # Hydrostatic quantities (Fossen 2021, Chapter 4)
        Aw_pont = Cw_pont * self.batched_params.L * self.batched_params.B_pont  # waterline area, one pontoon
        I_T = (
            2
            * (1 / 12)
            * self.batched_params.L
            * self.batched_params.B_pont ** 3
            * (6 * Cw_pont ** 3 / ((1 + Cw_pont) * (1 + 2 * Cw_pont)))
            + 2 * Aw_pont * y_pont ** 2
        )
        I_L = 0.8 * 2 * (1 / 12) * self.batched_params.B_pont * self.batched_params.L ** 3
        KB = (1 / 3) * (5 * self.batched_params.T / 2 - 0.5 * nabla / (self.batched_params.L * self.batched_params.B_pont))
        BM_T = I_T / nabla  # BM values
        BM_L = I_L / nabla
        KM_T = KB + BM_T    # KM values
        KM_L = KB + BM_L
        KG = self.batched_params.T - self.batched_params.r_bg[:, 2]
        GM_T = KM_T - KG    # GM values
        GM_L = KM_L - KG

        G33 =  torch.from_numpy(np.array([ self.batched_params.mass 
                                                 for _ in range(self.batch_size)])).to(self.device)
        G44 = self.batched_params.rho * self.batched_params.g * nabla * GM_T
        G55 = self.batched_params.rho * self.batched_params.g * nabla * GM_L
        G_CF =torch.diag_embed(torch.stack([torch.zeros_like(G55), torch.zeros_like(G55), G33, 
                                      G44, G55, torch.zeros_like(G55)], dim=1))

        LCF = -0.2
        H = bp.Hmtrx(torch.from_numpy(np.array([np.array([LCF, 0.0, 0.0])
                                                                for _ in range(self.batch_size)])).to(self.device))  # transform G_CF from CF to CO
        self.batched_params.G = H.mT @ G_CF @ H

        # Natural frequencies
        w3 = torch.sqrt(G33 / self.batched_params.M[:, 2, 2])
        w4 = torch.sqrt(G44 / self.batched_params.M[:, 3, 3])
        w5 = torch.sqrt(G55 / self.batched_params.M[:, 4, 4])

        # Linear damping terms (hydrodynamic derivatives)
        Xu = -24.4 * self.batched_params.g / Umax.squeeze(-1)   # specified using the maximum speed
        Yv = -self.batched_params.M[:,1, 1]  / T_sway # specified using the time constant in sway
        Zw = -2 * 0.03 * w3 * self.batched_params.M[:, 2, 2]  # specified using relative damping
        Kp = -2 * 0.02 * w4 * self.batched_params.M[:, 3, 3]
        Mq = -2 * 0.04 * w5 * self.batched_params.M[:, 4, 4]
        Nr = -self.batched_params.M[:, 5, 5] / T_yaw  # specified by the time constant T_yaw

        self.batched_params.D = -torch.diag_embed(torch.stack([Xu, Yv, Zw, Kp, Mq, Nr], dim=1))

        # Propeller configuration/input matrix
        # B = self.batched_params.k_pos * torch.from_numpy(np.array([np.array([[1, 1], [-self.batched_params.l1, -self.batched_params.l2]])
        #                                                            for _ in range(self.batch_size) ])).double().to(self.device)
        # self.batched_params.TM_to_f = torch.linalg.inv(B)

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
            )  # shape (num_rotors, 3)
            # Unit thrust directions for each thruster
            thrust_dirs = torch.from_numpy(self.batched_params.rotor_dir_np[i]).double().to(self.device)

            # Compute moment arms (r × F)
            torque_arms = torch.cross(pos, thrust_dirs, dim=-1).T  # (3,2)

            # Build full 6x2 allocation matrix
            A = torch.vstack([
                thrust_dirs.T,   # Fx,Fy,Fz rows
                torque_arms,     # Mx,My,Mz rows
            ])

            alloc_mats.append(A)

        # Use pseudo-inverse for generality
        TM_to_f = torch.stack(alloc_mats).to(self.device)
        self.batched_params.TM_to_f = torch.linalg.pinv(TM_to_f)

        # Low-Level Control Gains
        self.batched_params.k_vx = torch.tensor([params['k_vx'] 
                                                for _ in range(self.batch_size)]).unsqueeze(-1).to(self.device)
        
        self.batched_params.kp_yaw = torch.from_numpy(np.array([params['kp_yaw'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kd_yaw = torch.from_numpy(np.array([params['kd_yaw'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
    

    
    def _compute_external_forces(self, s : Tensor, rpy : Tensor):
        nu = torch.cat([s['v'][self.idxs], s['w'][self.idxs]], dim=1).double().to(self.device)
        eta = torch.cat([s['x'][self.idxs], rpy], dim=1).double().to(self.device)


        self.surface_mask = torch.ones(self.batch_size, 1, device=self.device).double()
        z = s['x'][:,2].unsqueeze(-1)
        self.surface_mask[s['x'][:,2] > 0] = 0

        # Current velocities
        u_c = (self.batched_params.V_c * 
               torch.cos(self.batched_params.beta_c - rpy[..., 1])).unsqueeze(-1)  # current surge velocity
        v_c = (self.batched_params.V_c * 
               torch.sin(self.batched_params.beta_c- rpy[..., 1])).unsqueeze(-1)   # current sway velocity

        nu_c = torch.cat([u_c, v_c, 
                          torch.zeros((self.batch_size,4))], dim=1).double().to(self.device) # current velocity 
        nu_r = nu - nu_c                               # relative velocity    
        Dnu_c = torch.cat([nu[:, 5:6] * v_c, -nu[:, 5:6] * u_c, 
                           torch.zeros((self.batch_size,4))], dim=1).double().to(self.device) # current velocity 

        # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
        top= torch.cat([self.batched_params.mass * self.batched_params.hat_map(nu[:, 3:6]).permute(2, 0, 1), 
                       torch.zeros(self.batch_size, 3, 3, dtype=torch.float64)], dim=2)
        bottom = torch.cat([torch.zeros(self.batch_size, 3, 3, dtype=torch.float64), 
                       self.batched_params.hat_map( torch.einsum('bij,bj->bi',self.batched_params.Ig, 
                                                                 nu[:, 3:6])).permute(2, 0, 1)], dim=2)
        
        CRB_CG = torch.cat([top, bottom], dim=1)
        CRB = self.batched_params.H_rg.mT @ CRB_CG @ self.batched_params.H_rg
        CA  = bp.m2c(self.batched_params.MA, nu_r)
        
        C = CRB + CA

        tau_damp = torch.einsum('bij,bj->bi',self.batched_params.D, nu_r) 
        tau_damp[:, 5] = tau_damp[:, 5] - 10 * self.batched_params.D[:, 5, 5] * torch.abs(nu_r[:, 5]) * nu_r[:, 5]
        tau_crossflow = bp.cross_flow_drag(self.batched_params.rho, self.batched_params.L, self.batched_params.B_pont, nu_r, self.batched_params.T, self.batched_params.rho)
        tau = (
               tau_damp 
               + tau_crossflow 
               - torch.einsum('bij,bj->bi',C, nu_r)  
               - torch.einsum('bij,bj->bi',self.batched_params.G, eta) 
               ) * self.surface_mask
        
        # Hack damp
        # tau *= 0.1
        tau[:, 0] *= 0
        return (tau, Dnu_c) 
    
    def _compute_body_wrench(self, rotor_speeds : Tensor):
        """
        Computes the wrench acting on the rigid body based on the rotor speeds for thrust and airspeed
        for aerodynamic forces.
        The airspeed is represented in the body frame.
        The net force Ftot is represented in the body frame.
        The net moment Mtot is represented in the body frame.
        """

        # thrust magnitudes per rotor
        T = torch.zeros(self.batch_size, 3, rotor_speeds.shape[-1], 
                        device=self.device).double().to(self.device)
        T[:, 0, :] = self.batched_params.k_eta[self.idxs] * rotor_speeds **2 *torch.sign(rotor_speeds) #polinomy to get thrust
        FtotB = torch.sum(T, dim=2)

        # ----- Moments (mt) -----
        l1 = self.batched_params.l1       
        l2 = self.batched_params.l2   
        M = self.batched_params.k_m[self.idxs] * rotor_speeds**2 *torch.sign(rotor_speeds) # (B,4)

        mt_x = torch.zeros((self.batch_size, 1))               # roll
        mt_y = torch.zeros((self.batch_size, 1))               # pitch
        # mt_z = -l1 * M[:, 0] +l2 * M[:, 1] # yaw
        mt_z =  -(-M[:, 0] + M[:, 1]) # yaw
        MtotB = torch.stack([mt_x, mt_y, mt_z.unsqueeze(-1)], dim=1)  # (B,3)


        Fx = FtotB[:, 0].unsqueeze(-1)
        tau = torch.stack([
            Fx,
            torch.zeros_like(Fx),
            torch.zeros_like(Fx),
            torch.zeros_like(Fx),
            torch.zeros_like(Fx),
            MtotB[:, 2],
        ], dim=1)

        return tau
    
    
    def _s_dot_fn(self, s, cmd_rotor_speeds : Tensor):
        state = self.unpack_state(s, self.idxs, self.batch_size)

        R = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()
        rpy = roma.rotmat_to_euler('xyz', R)

        x_dot = state['v'][self.idxs]
        q_dot = tfrm.quat_dot_torch(state['q'][self.idxs], state['w'][self.idxs])

        F, Dnu_c = self._compute_external_forces(state, rpy)
        F = F.unsqueeze(-1)

        FMtotB = self._compute_body_wrench(cmd_rotor_speeds)
        FMtotB[:, 1] = FMtotB[:, 0] * torch.cos(rpy[..., -1]).unsqueeze(-1)

        # FMtotB[:, :3] = R @ FMtotB[:, :3]
        # FMtotB[:, 1] = FMtotB[:, 0] * torch.cos(rpy[...,-1]).unsqueeze(-1)

        Ftot = FMtotB
        # self.batched_params.Minv[:, :3, :3] = self.batched_params.Minv[:, :3, :3]
        nu_dot = Dnu_c + (self.batched_params.Minv[self.idxs] @ Ftot[self.idxs]).squeeze(-1)

        v_dot = nu_dot[:, :3]
        w_dot = nu_dot[:, 3:]

        # Pack into vector of derivatives.
        s_dot = torch.zeros((len(self.idxs), 13 + cmd_rotor_speeds.shape[1],), device=self.device)
        s_dot[:, 0:3] = x_dot
        s_dot[:, 3:6] = v_dot
        s_dot[:, 6:10] = q_dot
        s_dot[:, 10:13] = w_dot.squeeze(-1)

        return s_dot
        
    
    def get_cmd_motor_speeds(self, state : dict,  control : dict):  #control needs to be a tensor
        if self._control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]
    
        if self._control_abstraction == 'cmd_vel':
            # --- Extract state ---
            v = state['v'][self.idxs]          # linear velocity (B,3)
            w = state['w'][self.idxs]          # angular velocity (B,3)
            q = state['q'][self.idxs]          # quaternion orientation (B,4)

            # --- Control input ---
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz]
            goal[:, 1:] = 0 # only vx is available

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[self.idxs] - v
            a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)


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
            goal = control['cmd_ctrl'][self.idxs]  # [vx, vy, vz, yaw]
            goal[:, 1:3] = 0 # only vx is available

            # --- Velocity control (P controller on velocity error) ---
            v_err = goal[:, :3] - v
            a_des = self.batched_params.k_vx[self.idxs] * v_err  # desired acceleration
            F_des = self.batched_params.mass * a_des[self.idxs]             # desired force in N (B,3)


            # --- Yaw stabilization ---
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
            M_des[:, 2] = M_yaw

            # # # --- Apply yaw-aligned rotation to F_des ---
            # R_wb = roma.unitquat_to_rotmat(q).transpose(1,2)   # world->body
            # F_des = torch.bmm(R_wb, F_des.unsqueeze(-1)).squeeze(-1)


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

    # @map_states
    # def step(self, state : dict, control : dict):
    #     """
    #     Integrate dynamics forward from state given constant control for time t_step.
    #     Params:
    #         - state: dictionary containing keys ['x', 'v', 'q', 'w', rotor_speeds'], and values which
    #           are pytorch tensors with dtype double and which have a batch dimension
    #         - control: dictionary with keys depending on the chosen control mode. Values are torch 
    #         tensors, again with dtype double and with a batch dimension equal to the number of agents.
    #     """
    #     cmd_rotor_speeds = self.get_cmd_motor_speeds(control)
    #     cmd_rotor_speeds = torch.clip(
    #         cmd_rotor_speeds,
    #         self.batched_params.rotor_speed_min[self.idxs],
    #         self.batched_params.rotor_speed_max[self.idxs]
    #     )

    #     s = self.pack_state(state, self.batch_size, self.device)
    #     s_dot = self._s_dot_fn(s, cmd_rotor_speeds)
    #     v_dot = torch.zeros_like(state["v"])
    #     w_dot = torch.zeros_like(state["w"])
    #     v_dot[self.idxs] = s_dot[..., 3:6].double()
    #     w_dot[self.idxs] = s_dot[..., 10:13].double()

    #     state = self.unpack_state(s, self.idxs, self.batch_size)
    #     state['q'][self.idxs] = state['q'][self.idxs] / \
    #                             torch.norm(state['q'][self.idxs], dim=-1).unsqueeze(-1)

    #     return Transform.convert_NED_to_NWU(state['q'][self.idxs], v_dot, w_dot).cpu().tolist()    
    #     return torch.cat([v_dot, w_dot], dim=1).cpu().tolist()