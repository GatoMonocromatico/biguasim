import torch
import roma

import numpy as np

from torch import Tensor
    
class BatchedParams:
    """ 
    A container class for various multirotor params. 
    Parameters:
        batch_size: number of agents in the simulation.
        params: dictionary containing the parameters for each vehicle.
        device: the device to use for the simulation (e.g. torch.device('cuda')).
    """

    def __init__(self, batch_size : int, params : dict, device : str):

        #batch parameters
        self.batch_size = batch_size
        self.device = device

        if 'rho' in params.keys():
            self.rho = params['rho']
        
        self.g = 9.80665

        #physical parameters
        self.mass = params['mass']

        self.weight = torch.zeros(batch_size, 3, device=device).double()
        self.weight[:,-1] = -self.mass * self.g
        
        # Rotor parameters
        self.rotor_speed_min = torch.tensor([params['rotor_speed_min'] for _ in range(batch_size)]).unsqueeze(-1).to(device) # rad/s air
        self.rotor_speed_max = torch.tensor([params['rotor_speed_max'] for _ in range(batch_size)]).unsqueeze(-1).to(device) # rad/s water

        if 'k_eta' in params.keys() and 'k_m' in params.keys():
            self.k_eta = torch.from_numpy(np.array([params['k_eta'] for _ in range(batch_size)])).unsqueeze(-1).to(device)     # thrust coeff, N/(rad/s)**2
            self.k_m = torch.from_numpy(np.array([params['k_m'] for _ in range(batch_size)])).unsqueeze(-1).to(device)
            
    
    @staticmethod
    def hat_map(s : Tensor):
        """
        Given vector s in R^3, return associate skew symmetric matrix S in R^3x3
        In the vectorized implementation, we assume that s is in the shape (N arrays, 3)
        """
        device = s.device
        if len(s.shape) > 1:  # Vectorized implementation
            s = s.unsqueeze(-1)
            hat = torch.cat([torch.zeros(s.shape[0], 1, device=device), -s[:, 2], s[:, 1],
                             s[:, 2], torch.zeros(s.shape[0], 1, device=device), -s[:, 0],
                             -s[:, 1], s[:, 0], torch.zeros(s.shape[0], 1, device=device)], dim=0).view(3, 3, s.shape[
                0]).double()
            return hat
        else:
            return torch.tensor([[0, -s[2], s[1]],
                                 [s[2], 0, -s[0]],
                                 [-s[1], s[0], 0]], device=device)
    @staticmethod  
    def Hmtrx(r : Tensor):
        """
        H = Hmtrx(r) computes the 6x6 system transformation matrix
        H = [eye(3)     S'
            zeros(3,3) eye(3) ]       Property: inv(H(r)) = H(-r)

        If r = r_bg is the vector from the CO to the CG, the model matrices in CO and
        CG are related by: M_CO = H(r_bg)' * M_CG * H(r_bg). Generalized position and
        force satisfy: eta_CO = H(r_bg)' * eta_CG and tau_CO = H(r_bg)' * tau_CG 
        """
        H = torch.eye(6).unsqueeze(0).repeat(r.shape[0], 1, 1).double().to(r.device)
        H[:, 0:3, 3:6] = BatchedParams.hat_map(r).mT.permute(1, 0, 2)

        return H
    

    @staticmethod
    def m2c(M, nu):
        """
        C = m2c(M,nu) computes the Coriolis and centripetal matrix C from the
        mass matrix M and generalized velocity vector nu (Fossen 2021, Ch. 3)
        """
        M = 0.5 * (M + M.mT)     # systematization of the inertia matrix
        batch_size = M.shape[0]
        device = M.device


        v_dot = nu[:, :3]
        w_dot = nu[:, 3:]

        if v_dot.shape[-1] == 3:      #  6-DOF model
        
            M11 = M[:, 0:3,0:3]
            M12 = M[:, 0:3,3:6] 
            M21 = M12.mT
            M22 = M[:, 3:6,3:6] 

        
            dt_dv_dot = torch.einsum('bij,bj->bi',M11,v_dot) + \
                                            torch.einsum('bij,bj->bi',M12,w_dot)
            dt_dw_dot = torch.einsum('bij,bj->bi',M21,v_dot) + \
                                            torch.einsum('bij,bj->bi',M22,w_dot)

            C = torch.cat([
                torch.cat([torch.zeros((batch_size, 3, 3), device=device), 
                        -BatchedParams.hat_map(dt_dv_dot).permute(2, 0, 1)  ], 
                        dim=-1), 
                torch.cat([-BatchedParams.hat_map(dt_dv_dot).permute(2, 0, 1), 
                        -BatchedParams.hat_map(dt_dw_dot).permute(2, 0, 1)], 
                        dim=-1) ], 
            dim=-2)

        else:   # 3-DOF model (surge, sway and yaw)
                
            C = torch.zeros((batch_size, 6, 6)).to(device)
            C[:, 0,2] = -M[:, 1,1] * v_dot[:, 1] - M[:, 1,2] * w_dot[:, 2]
            C[:, 1,2] = M[:, 0,0] * v_dot[:, 0]
            C[:, 2,0] = -C[:, 0,2]       
            C[:, 2,1] = -C[:, 1,2]
        return C
    
    @staticmethod
    def cross_flow_drag(rho : float, L: float, B: torch.Tensor, T: torch.Tensor, nu_r: torch.Tensor) -> torch.Tensor:
        """
        Cross-flow drag (batch version)
        
        Inputs:
            L: float, body length
            B, T: tensors of shape (B,)
            nu_r: tensor of shape (B,6), relative velocity
        Returns:
            tau_crossflow: tensor of shape (B,6)
        """

        def Hoerner(B,T):
            """2D Hoerner cross-flow form coefficient"""
            DATA1 = np.array([
                    0.0109,0.1766,0.3530,0.4519,0.4728,0.4929,0.4933,0.5585,0.6464,0.8336,
                    0.9880,1.3081,1.6392,1.8600,2.3129,2.6000,3.0088,3.4508, 3.7379,4.0031 
                    ]) 
            DATA2 = np.array([
                1.9661,1.9657,1.8976,1.7872,1.5837,1.2786,1.2108,1.0836,0.9986,0.8796,
                0.8284,0.7599,0.6914,0.6571,0.6307,0.5962,0.5868,0.5859,0.5599,0.5593 
                ])
            CY_2D = np.interp(B/(2*T), DATA1, DATA2)
            return CY_2D
        
        batch_size = nu_r.shape[0]
        device = nu_r.device
        n = 20
        dx =  L / n

        # Example 2D drag coefficients, batch version
        Cd_2D = torch.from_numpy(np.array([Hoerner(B, T) for _ in range(batch_size)])).to(device)
        # Cd_2D = 0
        # Initialize
        Yh = 0
        Nh = 0
        xL = -L / 2

        for _ in range(n+1):
            v_r = nu_r[:, 1]  # sway velocity
            r = nu_r[:, 5]    # yaw rate
            Ucf = (v_r + xL*r).abs() * (v_r + xL*r)
            Yh -= 0.5 * rho * T * Cd_2D * Ucf * dx
            Nh -= 0.5 * rho * T * Cd_2D * xL * Ucf * dx
            xL += dx

        tau_crossflow = torch.hstack([
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),
            Yh.unsqueeze(1),
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),
            torch.zeros(batch_size, 1, device=nu_r.device, dtype=nu_r.dtype),
            Nh.unsqueeze(1),
        ])

        return tau_crossflow
    

    
    def _extract_geometry(self):
        """
        Extracts the geometry in self.rotors for efficient use later on in the computation of
        wrenches acting on the rigid body.
        The rotor_geometry is a tensor of shape (num_agents, n,3), where n is the number of rotors.
        Each row corresponds to the position vector of the rotor relative to the CoM.
        """
        assert hasattr(self, "rotor_pos"), ":rotor_pos need to be defined."

        geoms = []
        geom_hat_maps = []
        for i in range(self.batch_size):
            rotor_geometry = np.array([]).reshape(0, 3)
            for rotor in self.rotor_pos[i]:
                rp = self.rotor_pos[i][rotor]
                rotor_geometry = np.vstack([rotor_geometry, rp])

            geoms.append(rotor_geometry)
            geom_hat_maps.append(self.hat_map(torch.from_numpy(rotor_geometry.squeeze())).numpy())
        self.rotor_geometry = torch.from_numpy(np.array(geoms)).to(self.device)
        self.rotor_geometry_hat_maps = torch.from_numpy(np.array(geom_hat_maps)).to(self.device)       

    


class Transform:
    T_coord_sys = torch.tensor([
        [ 1,  0,  0 ],
        [ 0,  -1,  0],
        [ 0,  0, -1] ]).double()


    @staticmethod
    def quat_dot_torch(quat : Tensor, omega : Tensor) -> Tensor:
        """
        Parameters:
            quat, (...,[i,j,k,w])
            omega, angular velocity of body in body axes: (...,3)

        Returns
            duat_dot, (...,[i,j,k,w])

        """
        b = quat.shape[0]
        # Adapted from "Quaternions And Dynamics" by Basile Graf.
        (q0, q1, q2, q3) = (quat[...,0], quat[...,1], quat[...,2], quat[...,3])
        G = torch.stack([q3, q2, -q1, -q0,
                         -q2, q3, q0, -q1,
                         q1, -q0, q3, -q2], dim=1).view((b, 3, 4))

        quat_dot = 0.5 * torch.transpose(G, 1,2) @ omega.unsqueeze(-1)
        # Augment to maintain unit quaternion.
        quat_err = torch.sum(quat**2, dim=-1) - 1
        quat_err_grad = 2 * quat
        quat_dot = quat_dot.squeeze(-1) - quat_err.unsqueeze(-1) * quat_err_grad
        return quat_dot
    
    @classmethod
    def _convert_NWU_to_NED(cls, x: torch.Tensor):
        """
        Convert full BiguaSim dynamic sensor output from NWU to NED coordinate system.

        Args:
            x (torch.Tensor): [B, 19] input (accel, vel, pos, ang_accel, ang_vel, quat)

        Returns:
            x_ned (torch.Tensor): [B, 19] output, converted to NED
        """
        B = x.shape[0]
        T = cls.T_coord_sys  # [3, 3]

        x_ned = x.clone()

        # Convert all 3D vectors using T
        for start, end in [(0, 3), (3, 6), (6, 9), (9, 12), (12, 15)]:
            vec = x[:, start:end]   # [B, 3]
            vec_ned = torch.matmul(T, vec.T).T  # [B, 3]
            x_ned[:, start:end] = vec_ned

        # Convert quaternion
        quat_nwu = x[:, 15:]                       # [B, 4]
        R_nwu = roma.unitquat_to_rotmat(quat_nwu)        # [B, 3, 3]
        T_expand = T.unsqueeze(0).expand(B, -1, -1)       # [B, 3, 3]
        R_ned = T_expand @ R_nwu @ T_expand               # [B, 3, 3]
        quat_ned = roma.rotmat_to_unitquat(R_ned)        # [B, 4]
        x_ned[:, 15:] = quat_ned

        return x_ned
    
    @classmethod
    def _convert_NED_to_NWU(cls, q: torch.Tensor, v_dot: torch.Tensor, w_dot: torch.Tensor) -> torch.Tensor:
        """
        Converts batched body-frame linear and angular acceleration from NED → NWU and 
        body → world frame using quaternion rotation.

        Args:
            q (torch.Tensor): [B, 4] batch of quaternions
            v_dot (torch.Tensor): [B, 3] linear acceleration in body frame (NED)
            w_dot (torch.Tensor): [B, 3] angular acceleration in body frame (NED)

        Returns:
            accel_nwu_world (torch.Tensor): [B, 6] linear + angular accel in world frame (NWU)
        """
        B = q.shape[0]
        T = cls.T_coord_sys  # [3, 3]


        
        # Convert quaternion to rotation matrices
        R_body_to_world = roma.unitquat_to_rotmat(q)  # [B, 3, 3]
        # Apply NED → NWU conversion and rotate to world frame
        T_expand = T.unsqueeze(0).expand(B, -1, -1)  # [B, 3, 3]

        # Transform: a_world = R * (T * a_body_NED)
        v_nwu_body = torch.matmul(T_expand, v_dot.unsqueeze(-1)).squeeze(-1)  # [B, 3]
        w_nwu_body = torch.matmul(T_expand, w_dot.unsqueeze(-1)).squeeze(-1)  # [B, 3]

        v_nwu_world = torch.bmm(R_body_to_world, v_nwu_body.unsqueeze(-1)).squeeze(-1)  # [B, 3]
        w_nwu_world = torch.bmm(R_body_to_world, w_nwu_body.unsqueeze(-1)).squeeze(-1)  # [B, 3]

        return torch.cat([v_nwu_world, w_nwu_world], dim=1)  # [B, 6]
    
    @classmethod
    def convert_NWU_to_NED(cls, pos : Tensor, quat : Tensor, vel : Tensor, omega : Tensor):
        """
        Converts sensor state from NWU to NED and world → body frames.

        Args:
            x: torch.Tensor of shape (..., 19)
               - x[..., 6:9]: position
               - x[..., 3:6]: velocity in world frame
               - x[..., 12:15]: angular velocity in world frame
               - x[..., 15:19]: quaternion (xyzw)

        Returns:
            eta, nu: (… x 6) tensors:
                eta = [x, y, z, roll, pitch, yaw]
                nu  = [u, v, w, p, q, r]
        """

        def R2euler(R: torch.Tensor):
            """
            Computes Euler angles (ZYX / roll-pitch-yaw) from a rotation matrix.
            Supports batch of rotation matrices: R shape (B,3,3) or single (3,3)

            Returns:
                phi, theta, psi: tensors of shape (B,1) if batch, else scalars
            """
            phi = torch.atan2(R[..., 2, 1], R[..., 2, 2])
            theta = -torch.asin(R[..., 2, 0])
            psi = torch.atan2(R[..., 1, 0], R[..., 0, 0])
            return phi, theta, psi
        

        T_coord_sys = cls.T_coord_sys.to(pos.device)
        R = roma.unitquat_to_rotmat(quat)
        R_inv = R.transpose(-1, -2)

        # Position: NWU -> NED
        eta_pos = torch.matmul(T_coord_sys, pos.unsqueeze(-1)).squeeze(-1)

        # Euler angles
        R_Fos = torch.matmul(torch.matmul(T_coord_sys, R), T_coord_sys)
        phi, theta, psi = R2euler(R_Fos)
        eta = torch.cat([eta_pos, phi.unsqueeze(-1), theta.unsqueeze(-1), psi.unsqueeze(-1)], dim=-1)

        # Velocities: world -> body, NWU -> NED
        vel_body_NED = torch.matmul(T_coord_sys, torch.matmul(R_inv, vel.unsqueeze(-1))).squeeze(-1)
        omega_body_NED = torch.matmul(T_coord_sys, torch.matmul(R_inv, omega.unsqueeze(-1))).squeeze(-1)
        nu = torch.cat([vel_body_NED, omega_body_NED], dim=-1)

        return eta, nu
    
    @classmethod
    def convert_NED_to_NWU(cls, quat_xyzw : Tensor, accel: Tensor):
        """
        Converts linear and angular acceleration from body frame NED to world frame NWU.
        
        Args:
            x: torch.Tensor of shape (...,19), containing sensor output including quaternion
               - x[..., 15:19] = quaternion (xyzw)
            accel: torch.Tensor of shape (...,6)
               - linear accel: accel[..., :3]
               - angular accel: accel[..., 3:6] (p_dot, q_dot, r_dot)
        
        Returns:
            accel_world: torch.Tensor (...,6) - accelerations in NWU world frame
        """
        T_coord_sys = cls.T_coord_sys.to(quat_xyzw.device)
        # Rotation matrix body → world
        R_body_to_world = roma.unitquat_to_rotmat(quat_xyzw)  # (...,3,3)

        # Linear and angular accelerations in body frame (NED)
        lin_accel = accel[..., :3].unsqueeze(-1)  # (...,3,1)
        ang_accel = accel[..., 3:6].unsqueeze(-1)  # (...,3,1)

        # Transform to world frame NWU
        lin_accel_world = torch.matmul(R_body_to_world, torch.matmul(T_coord_sys, lin_accel)).squeeze(-1)
        ang_accel_world = torch.matmul(R_body_to_world, torch.matmul(T_coord_sys, ang_accel)).squeeze(-1)

        # Concatenate linear and angular accelerations
        accel_world = torch.cat([lin_accel_world, ang_accel_world], dim=-1)

        return accel_world
    



    




















