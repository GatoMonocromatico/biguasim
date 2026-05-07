import torch
import roma

import numpy as np

from torch import Tensor

from biguasim.dynamics.base_model import VehicleModel
from biguasim.dynamics.control import BatchedPIDController
from biguasim.dynamics.utils import Transform as tfrm



class QuadCopterX(VehicleModel):
    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:
        super().__init__(batch_size, params, device, control_abstraction)   

        self._build_params(params)

        self.linear_pid  = BatchedPIDController(Kp=10, Ki=1, Kd=0.1,
                                        batch_shape=(batch_size, 3), device=device)
        self.angular_pid = BatchedPIDController(Kp=10, Ki=1, Kd=0.1,
                                        batch_shape=(batch_size, 3), device=device)
        

    def _build_params(self, params : dict):
        self.batched_params.num_rotors = len(params["rotor_pos"].keys())
        self.batched_params.rotor_pos = [params["rotor_pos"] for _ in range(self.batch_size)]
        self.batched_params.rotor_dir_np = np.array([params['rotor_directions'] for _ in range(self.batch_size)]) #air
        self.batched_params._extract_geometry()

        self.batched_params.rotor_dir = torch.from_numpy(self.batched_params.rotor_dir_np).to(self.device)

        self.batched_params.drag_matrix = torch.tensor([[[params["c_Dx"],   0,                 0],
                                        [0,          params["c_Dy"],          0],
                                        [0,          0,          params["c_Dz"]]] for _ in range(self.batch_size)], 
                                                                                        device=self.device).double()
        
        self.batched_params.I = torch.from_numpy(np.array([params['I'] 
                                                        for _ in range(self.batch_size)])).double().to(self.device)
        self.batched_params.invI = torch.linalg.inv(self.batched_params.I).double()

        # Low-Level Control Gains
        self.batched_params.k_v = torch.tensor([params['k_v'] 
                                                for _ in range(self.batch_size)]).unsqueeze(-1).to(self.device)
        

        
        k = self.batched_params.k_m/self.batched_params.k_eta  # Ratio of torque to thrust coefficient.

        alloc_mats = []
        for i in range(self.batch_size):
            # Convert numpy -> torch
            pos = torch.tensor(
                np.stack([self.batched_params.rotor_pos[i][key] for key in self.batched_params.rotor_pos[i]], axis=0),
                dtype=torch.float64
            ).to(self.device)  # shape (num_rotors, 3)

            # Cross product with z-axis
            z_axis = torch.tensor([0.,0.,1.], dtype=torch.float64).to(self.device)
            cross = torch.cross(pos, z_axis.expand_as(pos)).to(self.device)

            # Keep XY components only
            cross_xy = cross[:, :2].T   # shape (2, num_rotors)

            # Build matrix
            mat = torch.vstack([
                torch.ones((1, len(self.batched_params.rotor_pos[i])), dtype=torch.float64).to(self.device),        # thrust row
                cross_xy,                                               # roll & pitch
                (k[i] * self.batched_params.rotor_dir[i]).reshape(1, -1)                 # yaw torque
            ]).to(self.device)
            alloc_mats.append(mat)

        
        self.batched_params.TM_to_f = torch.linalg.inv(torch.stack(alloc_mats).to(self.device))

        # Control allocation
        self.batched_params.kp_att = torch.from_numpy(np.array([params['kp_att'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kd_att = torch.from_numpy(np.array([params['kd_att'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        
        self.batched_params.kp_yaw = torch.from_numpy(np.array([params['kp_yaw'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kd_yaw = torch.from_numpy(np.array([params['kd_yaw'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)

        
        self.batched_params.kp_pos = torch.from_numpy(np.array([params['kp_pos'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        self.batched_params.kd_pos = torch.from_numpy(np.array([params['kd_pos'] 
                                                                for _ in range(self.batch_size)])).unsqueeze(-1).to(self.device)
        
        
    def _compute_external_forces(self, R : Tensor, inertial_velocity : Tensor):
        body_airspeed_vector = R.transpose(1, 2) @ (inertial_velocity).unsqueeze(-1).double()
        body_airspeed_vector = body_airspeed_vector.squeeze(-1)

        tmp = self.batched_params.drag_matrix[self.idxs] @ (body_airspeed_vector).unsqueeze(-1)
        D = -torch.linalg.norm(body_airspeed_vector, dim=-1).unsqueeze(-1) * tmp.squeeze()

        return D


    def _compute_body_wrench(self, rotor_speeds): 
        """
        Computes the wrench acting on the rigid body based on the rotor speeds for thrust and airspeed
        for aerodynamic forces.
        The airspeed is represented in the body frame.
        The net force Ftot is represented in the body frame.
        The net moment Mtot is represented in the body frame.
        """
        T = torch.zeros(self.batch_size, 3, self.batched_params.num_rotors, device=self.device).double()
        T[..., -1, :] = self.batched_params.k_eta[self.idxs] * rotor_speeds** 2 #polinomy to get thrust

        M_force = -torch.einsum('bijk, bik->bj', self.batched_params.rotor_geometry_hat_maps[self.idxs], T)
        M_yaw = torch.zeros(self.batch_size, 3, 4, device=self.device)
        M_yaw[..., -1, :] = self.batched_params.rotor_dir[self.idxs] *\
                         self.batched_params.k_m[self.idxs] * rotor_speeds ** 2


        # Sum all elements to compute the total body wrench
        FtotB = torch.sum(T, dim=2)
        MtotB = M_force + torch.sum(M_yaw, dim=2)

        return (FtotB, MtotB)    
    

    def _s_dot_fn(self, s, cmd_ctrl):
        """
        Compute derivative of state for quadrotor given fixed control inputs as
        an autonomous ODE.
        """

        state = self.unpack_state(s, self.idxs, self.batch_size)
        inertial_velocity = state['v'][self.idxs]
        R = roma.unitquat_to_rotmat(state['q'][self.idxs]).double()

        x_dot = state['v'][self.idxs]
        q_dot = tfrm.quat_dot_torch(state['q'][self.idxs], state['w'][self.idxs])

        F = self._compute_external_forces(R, inertial_velocity)

        #compute body wrench
        (FtotB, MtotB) = self._compute_body_wrench(cmd_ctrl)
        FtotB += F
        Ftot = R @ FtotB.unsqueeze(-1)

        v_dot = (self.batched_params.weight[self.idxs] + Ftot.squeeze(-1)) / self.batched_params.mass
        w = state['w'][self.idxs].double() 
        w_hat = self.batched_params.hat_map(w).permute(2, 0, 1)
        w_dot = self.batched_params.invI[self.idxs] @ (
                    MtotB - (w_hat.double() @ (self.batched_params.I[self.idxs] @ w.unsqueeze(-1))).squeeze(-1)).unsqueeze(-1)

        # Pack into vector of derivatives.
        s_dot = torch.zeros((len(self.idxs), 13 + self.batched_params.num_rotors,), device=self.device)
        s_dot[:, 0:3] = x_dot
        s_dot[:, 3:6] = v_dot
        s_dot[:, 6:10] = q_dot
        s_dot[:, 10:13] = w_dot.squeeze(-1)

        return s_dot
    
    
    def get_cmd_motor_speeds(self, state : dict, control : dict):  #control needs to be a tensor
        if self.control_abstraction == 'cmd_motor_speeds':
            return control['cmd_ctrl'][self.idxs]
        
        if self.control_abstraction == 'cmd_vel':
            # --- Velocity error ---
            v_err = state['v'][self.idxs] - control['cmd_ctrl'][self.idxs]

            # Desired acceleration from P-control
            a_cmd = -self.batched_params.k_v[self.idxs] * v_err

            # Desired total force including gravity
            F_des = self.batched_params.mass * (a_cmd + torch.tensor([0, 0, self.batched_params.g], device=self.device).double())
            R = roma.unitquat_to_rotmat(state['q'][self.idxs]).double() 
            b3 = R @ torch.tensor([0.0, 0.0, 1.0], device=self.device).double()
            cmd_thrust = torch.sum(F_des * b3, dim=-1).double().unsqueeze(-1)

            # Follow rest of SE3 controller to compute cmd moment.
            # Desired orientation to obtain force vector.
            tmp = torch.norm(F_des, dim=-1, keepdim=True)
            mask = (tmp.squeeze(-1) > 0)
            b3_des = torch.zeros_like(F_des)
            b3_des[mask] = F_des[mask] / torch.norm(F_des[mask], dim=-1, keepdim=True)
            
            c1_des = torch.tensor([1.0, 0.0, 0.0], device=self.device).repeat(self.batch_size, 1).double()    #original
            
    
            cross = torch.cross(b3_des, c1_des, dim=-1)
            tmp = torch.norm(cross, dim=-1, keepdim=True)
            mask = (tmp.squeeze(-1) > 0)
            b2_des = torch.zeros_like(cross)
            b2_des[mask] = cross[mask] / torch.norm(cross[mask], dim=-1, keepdim=True)
            b1_des = torch.cross(b2_des, b3_des, dim=-1)
            R_des = torch.stack([b1_des, b2_des, b3_des], dim=-1)

            # --- Orientation error ---
            S_err = 0.5 * (R_des.transpose(-1, -2) @ R - R.transpose(-1, -2) @ R_des)
            att_err = torch.stack([-S_err[:, 1, 2], S_err[:, 0, 2], -S_err[:, 0, 1]], dim=-1)

            # --- Angular control --
            Iw = self.batched_params.I[self.idxs] @ state['w'][self.idxs].unsqueeze(-1)
            tmp = -self.batched_params.kp_att[self.idxs] * att_err - self.batched_params.kd_att[self.idxs] * state['w'][self.idxs]
            cmd_moment = (self.batched_params.I[self.idxs] @ tmp.unsqueeze(-1)).squeeze(-1) \
                        + torch.cross(state['w'][self.idxs], Iw.squeeze(-1), dim=-1)
            
            # --- Map thrust & moments to rotor speeds ---
            TM = torch.cat([cmd_thrust, cmd_moment.squeeze(-1)], dim=-1)
            cmd_rotor_thrusts = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(1).transpose(-1, -2)).squeeze(-1)
            cmd_motor_speeds = cmd_rotor_thrusts / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            return cmd_motor_speeds
        

        if self.control_abstraction == 'cmd_vel_yaw':
            # --- Velocity error ---
            v_err = state['v'][self.idxs] - control['cmd_ctrl'][self.idxs][:, :3]

            # Desired acceleration (P control on velocity)
            a_cmd = -self.batched_params.k_v[self.idxs] * v_err

            yaw = roma.unitquat_to_rotvec(state['q'][self.idxs])[:, 2]  # current yaw angle (ψ)

            # --- Relative yaw command (in degrees) ---
            yaw_delta_deg = control['cmd_ctrl'][:, 3]   # e.g. 10 means +10 degrees
            yaw_delta = torch.deg2rad(yaw_delta_deg.double())     # convert to radians

            # Desired yaw = current yaw + relative offset
            yaw_des = yaw + yaw_delta

            # Wrap to [-π, π] for numerical stability
            yaw_des = (yaw_des + np.pi) % (2 * np.pi) - np.pi

            # --- Yaw control ---
            yaw_rate = state['w'][self.idxs][:, 2]  # current yaw rate (ω_z)
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            # Desired total force including gravity
            F_des = self.batched_params.mass * (
                a_cmd + torch.tensor([0, 0, self.batched_params.g], device=self.device).double()
            )

            r = torch.zeros_like(F_des)
            r[:, -1] = yaw_des
            r = roma.rotvec_to_unitquat(r)
            r = roma.quat_product(state['q'][self.idxs], r)
            R_r = roma.unitquat_to_rotmat(r).double()                # rotation using yaw_des
            R_q = roma.unitquat_to_rotmat(state['q'][self.idxs]).double() # rotation from state

            # -- Step 2: Create batch mask based on yaw_des --
            mask = (yaw_des.squeeze(-1) < -np.pi/2) | (yaw_des.squeeze(-1) > np.pi/2)
            mask = mask.unsqueeze(-1).unsqueeze(-1)  # [B,1,1] for broadcasting

            #-- Step 3: Select per batch element --
            R = torch.where(mask, R_r, R_q)

            b3 = R @ torch.tensor([0.0, 0.0, 1.0], device=self.device).double()
            cmd_thrust = torch.sum(F_des * b3, dim=-1).double().unsqueeze(-1)

            # PD yaw control
            yaw_dot_des = self.batched_params.kp_yaw[self.idxs].squeeze(-1) * yaw_err
            yaw_dot_err = yaw_dot_des - yaw_rate
            M_yaw = self.batched_params.kd_yaw[self.idxs].squeeze(-1) * yaw_dot_err

            # --- Desired orientation (roll/pitch from F_des, yaw from yaw_des) ---
            tmp = torch.norm(F_des, dim=-1, keepdim=True)
            mask = (tmp.squeeze(-1) > 0)
            b3_des = torch.zeros_like(F_des)
            b3_des[mask] = F_des[mask] / torch.norm(F_des[mask], dim=-1, keepdim=True)
            c1_des = torch.stack(
                [torch.cos(yaw_des), torch.sin(yaw_des), torch.zeros_like(yaw_des)],
                dim=-1
            )

            cross = torch.cross(b3_des, c1_des, dim=-1)
            tmp = torch.norm(cross, dim=-1, keepdim=True)
            mask = (tmp.squeeze(-1) > 0)
            b2_des = torch.zeros_like(cross)
            b2_des[mask] = cross[mask] / torch.norm(cross[mask], dim=-1, keepdim=True)

            b1_des = torch.cross(b2_des, b3_des, dim=-1)
            R_des = torch.stack([b1_des, b2_des, b3_des], dim=-1)

            # --- Orientation error ---
            S_err = 0.5 * (R_des.transpose(-1, -2) @ R - R.transpose(-1, -2) @ R_des)
            att_err = torch.stack([-S_err[:, 1, 2], S_err[:, 0, 2], -S_err[:, 0, 1]], dim=-1)

            # --- Angular control ---
            Iw = self.batched_params.I[self.idxs] @ state['w'][self.idxs].unsqueeze(-1)
            tmp = -self.batched_params.kp_att[self.idxs] * att_err - self.batched_params.kd_att[self.idxs] * state['w'][self.idxs]
            cmd_moment = (self.batched_params.I[self.idxs] @ tmp.unsqueeze(-1)).squeeze(-1) \
                        + torch.cross(state['w'][self.idxs], Iw.squeeze(-1), dim=-1)

            # Add yaw moment correction
            cmd_moment[:, 2] += M_yaw[self.idxs]

            # --- Map thrust & moments to rotor speeds ---
            TM = torch.cat([cmd_thrust, cmd_moment.squeeze(-1)], dim=-1)
            cmd_rotor_thrusts = (self.batched_params.TM_to_f[self.idxs] @ TM.unsqueeze(1).transpose(-1, -2)).squeeze(-1)
            cmd_motor_speeds = cmd_rotor_thrusts / self.batched_params.k_eta[self.idxs]
            cmd_motor_speeds = torch.sign(cmd_motor_speeds) * torch.sqrt(torch.abs(cmd_motor_speeds))

            return cmd_motor_speeds
        

        if self.control_abstraction == 'cmd_pos_yaw':
            """
            PD position + yaw controller that outputs rotor speeds,
            using cmd_ctrl as input, self.batched_params, self.idxs, self.device.
            """
            bp = self.batched_params
            idxs = self.idxs
            device = self.device

            # --- Desired position & yaw ---
            pos_des = control['cmd_ctrl'][idxs, :3].double()  # [B,3]
            yaw_des = control['cmd_ctrl'][idxs, 3].double()  # [B,3]
            yaw_des = torch.deg2rad(yaw_des)
            
            # --- Position PD ---
            x = state['x'][idxs].double()
            v = state['v'][idxs].double()

            yaw = roma.unitquat_to_rotvec(state['q'][idxs])[:, 2]  # current yaw angle (ψ)
            e_pos = pos_des - x
            e_vel = -v
            F_des = bp.kp_pos[idxs] * e_pos + bp.kd_pos[idxs] * e_vel \
                    + torch.tensor([0, 0, self.batched_params.g], device=self.device).double() # [B,3] 
            F_des = F_des * bp.mass               # [B,3]
            yaw_rate = state['w'][self.idxs][:, 2]      # current yaw rate (ω_z)

            # Wrap yaw error to [-π, π]
            yaw_err = (yaw_des - yaw + np.pi) % (2 * np.pi) - np.pi

            # --- Current rotation ---
            r = torch.zeros_like(F_des)
            r[:, -1] = yaw_des
            r = roma.rotvec_to_unitquat(r)
            r = roma.quat_product(state['q'][idxs], r)
            R_r = roma.unitquat_to_rotmat(r).double()                # rotation using yaw_des
            R_q = roma.unitquat_to_rotmat(state['q'][idxs]).double() # rotation from state

            # -- Step 2: Create batch mask based on yaw_des --
            mask = (yaw_des.squeeze(-1) < -np.pi/2) | (yaw_des.squeeze(-1) > np.pi/2)
            mask = mask.unsqueeze(-1).unsqueeze(-1)  # [B,1,1] for broadcasting

            # -- Step 3: Select per batch element --
            R = torch.where(mask, R_r, R_q)
            b3 = torch.einsum("bij,j->bi", R, torch.tensor([0.0,0.0,1.0], device=device).double())
            cmd_thrust = torch.sum(F_des * b3, dim=-1).unsqueeze(-1)

            # --- Desired orientation with yaw ---
            b3_des = F_des / torch.norm(F_des, dim=-1, keepdim=True)
            # Reference x-axis in world frame based on yaw_des
            c1_des = torch.stack([torch.cos(yaw_err), torch.sin(yaw_err), torch.zeros_like(yaw)], dim=-1) 
            

            cross = torch.cross(b3_des, c1_des, dim=-1)
            tmp = torch.norm(cross, dim=-1, keepdim=True)
            mask = (tmp.squeeze(-1) > 0)
            b2_des = torch.zeros_like(cross)
            b2_des[mask] = cross[mask] / torch.norm(cross[mask], dim=-1, keepdim=True)

            b1_des = torch.cross(b2_des, b3_des, dim=-1)
            R_des = torch.stack([b1_des, b2_des, b3_des], dim=-1)

            # --- Attitude error ---
            S_err = 0.5 * (torch.matmul(R_des.transpose(1,2), R) - torch.matmul(R.transpose(1,2), R_des))
            att_err = torch.stack([-S_err[:,1,2], S_err[:,0,2], -S_err[:,0,1]], dim=-1)

            # --- Desired moments ---
            # PD yaw control law
            M_yaw = self.batched_params.kp_att[self.idxs].squeeze(-1) * yaw_err - self.batched_params.kd_att[self.idxs].squeeze(-1) * yaw_rate

            I = bp.I[idxs].double()
            Iw = torch.einsum("bij,bi->bj", I, state['w'][idxs])
            tmp = -bp.kp_att[idxs] *  att_err - bp.kd_att[idxs] * state['w'][idxs] 
            cmd_moment = torch.einsum("bij,bi->bj", I, tmp) + torch.cross(state['w'][idxs], Iw, dim=-1)
            cmd_moment[:, 2] += M_yaw

            # --- Stack thrust + moments ---
            TM = torch.cat([cmd_thrust, cmd_moment], dim=-1)  # [B,4]

            # --- Map to rotor thrusts ---
            cmd_rotor_thrusts = torch.einsum("bij,bj->bi", bp.TM_to_f[idxs], TM)

            # --- Convert rotor thrusts to motor speeds ---
            k_eta = bp.k_eta[idxs]
            cmd_motor_speeds = torch.sign(cmd_rotor_thrusts) * torch.sqrt(torch.abs(cmd_rotor_thrusts / k_eta))            
            return cmd_motor_speeds # [B,4] rad/s
        

        