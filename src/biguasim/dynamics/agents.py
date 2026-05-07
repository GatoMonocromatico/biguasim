import numpy as np

from biguasim.dynamics import uav, uuv, usv

    
class BlueBoat(usv.Catamaran):                                                                   
    _params = {
        # 'rho' : 1026,      # Water density
        'rho' : 1026,

        'mass' : 14.5,
        #Params to each vessel
        'length' : 1.195,      # m
        'width' : 0.17,      # m
        'height' : 0.376,      # m

        'I' : np.diag([0.5, 10, 11]),

         'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([-0.5475, 0.285, -0.276]),        # Rotor 1 position
            'r2': np.array([-0.5475, -0.285, -0.276]),         # Rotor 2 position            
        },

        'rotor_directions': np.array([[1, 0, 0],
                                      [1, 0, 0]]),


        'k_eta' : 3.55e-4,  # Rotor torque constant
        'k_m' : 1.12e-5,    # Rotor momentum constant

        'rotor_speed_min': -295.75,   # minimum rotor speed, rad/s
        'rotor_speed_max': 288.08,    # maximum rotor speed, rad/s


        # Lower level controller properties (for higher level control abstractions)
        'k_vx': 1,            # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_yaw': 1.2,            # The attitude P gain (for cmd_vel_yaw and cmd_pos_yaw)
        'kd_yaw': 1.2,            # The attitude D gain (for cmd_vel_yaw and cmd_pos_yaw)

        'kp_pos': .1,            # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': .1,            # The attitude D gain (for cmd_pos_yaw)

    }            

    _scheme = 2
                                                                                               
                                                                                               
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or BlueBoat._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params     
    
class BlueROV2(uuv.HexaCopterFiveDoF):                                                                   
    _params = {
        'rho' : 997,      # Water density

        #spheroid
        'mass' : 10.5,
        'length' : 0.4571,      # m
        'width' : 0.3381,      # m
        'height' : 0.2539,      # m

        'I' : np.diag([0.26, 0.23, 0.37]),

        'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([0.0, 0.22, 0.08]),        # Rotor 1 position
            'r2': np.array([0.0, -0.22, 0.08]),         # Rotor 2 position
            'r3': np.array([0.156, 0.111, 0.045]),    # Rotor 3 position
            'r4': np.array([0.156, -0.111, 0.045]),     # Rotor 4 position
            'r5': np.array([-0.156, 0.111, 0.045]),      # Rotor 5 position
            'r6': np.array([-0.156, -0.111, 0.045]),     # Rotor 6 position
            
        },

        'rotor_directions': np.array([[0, 0, 1],
                            [0, 0, 1],
                            [np.cos(7*np.pi/4), np.sin(7*np.pi/4), 0],
                            [np.cos(np.pi/4), np.sin(np.pi/4), 0],
                            [np.cos(5*np.pi/4), np.sin(5*np.pi/4), 0],
                            [np.cos(3*np.pi/4), np.sin(3*np.pi/4), 0]]),
        
        

        'k_eta' : 3.8e-4,  # Rotor torque constant
        'k_m' : 5.3e-6,    # Rotor momentum constant

        'rotor_speed_min': -278.9,   # minimum rotor speed, rad/s
        'rotor_speed_max': 278.9, # maximum rotor speed, rad/s

        # Lower level controller properties (for higher level control abstractions)
        'k_vxy': 1,           # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'k_vz': 60.0,           # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_roll': 10,         # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'ki_roll': .1,         # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'kd_roll': .01,        # The attitude D gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_yaw': 1,            # The attitude P gain (for cmd_vel_yaw and cmd_pos_yaw)
        'kd_yaw': 1,          # The attitude D gain (for cmd_vel_yaw and cmd_pos_yaw)

        'kp_pos': 1,            # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': 1,          # The attitude D gain (for cmd_pos_yaw)

    }            

    _scheme = 2                                                                               
                                                                                               
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or BlueROV2._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params                 
                             

class BlueROVHeavy(uuv.OctaCopterSixDoF):                                                                   
    _params = {
        'rho' : 997,      # Water density

        #spheroid
        'mass' : 11.5,
        'length' : 0.4571,      # m
        'width' : 0.3381,      # m
        'height' : 0.5539,      # m

        'I' : np.diag([0.26, 0.23, 0.37]),

        'rotor_pos': {          # location of each rotor in meters
            'r1': np.array([0.22, 0.22, 0.08]),          # Rotor 1 position
            'r2': np.array([0.22, -0.22, 0.08]),         # Rotor 2 position
            'r3': np.array([-0.22, 0.22, 0.08]),         # Rotor 3 position
            'r4': np.array([-0.22, -0.22, 0.08]),        # Rotor 4 position
            'r5': np.array([0.156, 0.111, 0.045]),       # Rotor 5 position
            'r6': np.array([0.156, -0.111, 0.045]),      # Rotor 6 position
            'r7': np.array([-0.156, 0.111, 0.045]),      # Rotor 7 position
            'r8': np.array([-0.156, -0.111, 0.045]),     # Rotor 8 position
            
        },

        'rotor_directions': np.array([[0, 0, 1],
                                    [0, 0, 1],
                                    [0, 0, 1],
                                    [0, 0, 1],
                                    [np.cos(7*np.pi/4), np.sin(7*np.pi/4), 0],
                                    [np.cos(np.pi/4), np.sin(np.pi/4), 0],
                                    [np.cos(5*np.pi/4), np.sin(5*np.pi/4), 0],
                                    [np.cos(3*np.pi/4), np.sin(3*np.pi/4), 0]     ]),

        'k_eta' : 3.8e-4,  # Rotor torque constant
        'k_m' : 5.3e-6,    # Rotor momentum constant

        'rotor_speed_min': -278.9,   # minimum rotor speed, rad/s
        'rotor_speed_max': 278.9, # maximum rotor speed, rad/s

        # Lower level controller properties (for higher level control abstractions)
        'k_vxy': 1,            # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw)
        'k_vz': 30,            # The *world* velocity P gain (for cmd_vel, cmd_vel_yaw)

        'kp_roll': 10,         # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'ki_roll': .1,         # The attitude I gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'kd_roll': .01,        # The attitude D gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_pitch': 10,         # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'ki_pitch': .1,         # The attitude I gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'kd_pitch': .01,        # The attitude D gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        
        'kp_yaw': 1,           # The attitude P gain (for cmd_vel_yaw and cmd_pos_yaw)
        'kd_yaw': 1,           # The attitude D gain (for cmd_vel_yaw and cmd_pos_yaw)

        'kp_pos': 1,           # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': 1,           # The attitude D gain (for cmd_pos_yaw)
                                                                                               
    }

    _scheme = 2
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or BlueROVHeavy._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return BlueROVHeavy._params      

class DjiMatrice(uav.QuadCopterX):
    _params = {
        # Inertial properties
        'mass': 3.8,            # kg, approximate weight of DJI Matrice quadcopter

        'rho' : 1225,       # Air density

        'I' : np.diag([0.07, 0.07,  0.13]),
        
        # Geometric properties, all vectors relative to center of mass
        'd' : 0.33,             # Arm length    

        'rotor_pos': {          # location of each rotor in meters
            'r1': 0.33 * np.array([0.70710678118, 0.70710678118, 0]),       # Rotor 1 position
            'r2': 0.33 * np.array([0.70710678118, -0.70710678118, 0]),       # Rotor 2 position
            'r3': 0.33 * np.array([-0.70710678118, -0.70710678118, 0]),      # Rotor 3 position
            'r4': 0.33 * np.array([-0.70710678118, 0.70710678118, 0]),      # Rotor 4 position
        },
        
        'k_eta' : 3.55e-4, 
        'k_m' : 1.12e-5,   

        'rotor_directions': np.array([1, -1, 1, -1]),  # Rotor spin directions (+1 for CW, -1 for CCW)
        'rotor_speed_min': 0,   # minimum rotor speed, rad/s
        'rotor_speed_max': 592.4, # maximum rotor speed, rad/s

        # Frame aerodynamic properties
        'c_Dx': 0.1,            # parasitic drag coefficient in body x-axis, N/(m/s)^2
        'c_Dy': 0.1,            # parasitic drag coefficient in body y-axis, N/(m/s)^2
        'c_Dz': 0.15,           # parasitic drag coefficient in body z-axis, N/(m/s)^2

        # Lower level controller properties (for higher level control abstractions)
        'k_v': 1,              # The *world* velocity P gain (for cmd_vel)
        'kp_att': 0.1,            # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'kd_att': 0.01,          # The attitude D gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_yaw': 1,            # The attitude P gain (for cmd_vel_yaw)
        'kd_yaw': 0.1,          # The attitude D gain (for cmd_vel_yaw)

        'kp_pos': 0.05,            # The attitude P gain (for cmd_pos_yaw)
        'kd_pos': 0.01,          # The attitude D gain (for cmd_pos_yaw)

    }

    _scheme = 1
    
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):
        super().__init__(
                        batch_size, 
                        params= params or DjiMatrice._params, 
                        device=device, 
                        control_abstraction=control_abstraction)
        
        self._params = params

    @property
    def params(self) -> dict: 
        return self._params     
                                                                                                    
class Torpedo(uuv.TorpedoAUV):
    _params = {
        "rho": 1026,

        "mass": 16,
        "l": 1.6,
        "d": 0.19,

        "r_bg": [0, 0, 0.02],
        "r_bb": [0, 0, 0],

        "r44": 0.3,
        "Cd": 0.42,
        "T_surge": 20,
        "T_sway": 20,
        "zeta_roll": 0.3,
        "zeta_pitch": 0.8,
        "T_yaw": 1,
        "K_nomoto": 5.0 / 20.0,

        #Actuator params:
        "fin_area": 0.00697,
        "fin_center": 0.07,
        "deltaMax_fin_deg": 20,
        "nMax": 2000,
        "T_delta": 0.1,
        "T_n": 0.1,
        "CL_delta_r": 0.5,
        "CL_delta_s": 0.7,

        'k_eta' : 3.55e-4,  # Rotor torque constant
        'k_m' : 1.12e-5,    # Rotor momentum constant

        'rotor_speed_min': 0,   # minimum rotor speed, rad/s
        'rotor_speed_max': 592.4, # maximum rotor speed, rad/s
    }

    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params = None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or TorpedoAUV._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params
    
class TorpedoAUV(uuv.TorpedoAUV):
    _params = {
        #Dynamics
        "mass": 16,
        "length": 1.6,
        "rho": 1026,
        "diam": 0.19,
        "r_bg": [0, 0, 0.02],
        "r_bb": [0, 0, 0],
        "r44": 0.3,
        "Cd": 0.42,
        "T_surge": 20,
        "T_sway": 20,
        "zeta_roll": 0.3,
        "zeta_pitch": 0.8,
        "T_yaw": 1,
        "K_nomoto": 5.0 / 20.0,

        #Actuador
        "fin_area": 0.00697,
        "fin_center": 0.07,
        "deltaMax_fin_deg": 20,
        "nMax": 2000,
        "T_delta": 0.1,
        "T_n": 0.1,
        "CL_delta_r": 0.5,
        "CL_delta_s": 0.7,
        
        #Control
        #depth
        'wn_d_z': 0.2,
        'Kp_z': 0.1,
        'T_z': 100,
        'Kp_theta': 5.0,
        'Kd_theta': 2.0,
        'Ki_theta': 0.3,
        'K_w':  5.0,
        'theta_max_deg': 30,

        #heading
        'wn_d': 1.2,
        'zeta_d': 0.8,
        'r_max': 0.9,
        'lam': 0.1,
        'phi_b': 0.1,
        'K_d': 0.5,
        'K_sigma': 0.05,

        #surge
        'kp_surge': 400.0,
        'ki_surge': 50.0,
        'kd_surge': 30.0,

        "rotor_speed_min": -1525, #rpm
        "rotor_speed_max": 1525, #rpm
    }

    _scheme = 1

    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params = None):    
        super().__init__(                                                                      
                        batch_size,                                                            
                        params= params or TorpedoAUV._params,                                       
                        device=device,                                                         
                        control_abstraction=control_abstraction)                               
                                                                                               
    @property                                                                                  
    def params(self) -> dict:                                                                  
        return self._params

class ModelsFactory:
    _types = {
        'BlueBoat' : BlueBoat,
        'BlueROV2' : BlueROV2,
        'BlueROVHeavy' : BlueROVHeavy,
        'DjiMatrice' : DjiMatrice,
        'TorpedoAUV' : TorpedoAUV
    }

    @classmethod
    def build_model(cls, agent_type : str):
        return cls._types[agent_type]


#começar por aqui
        



























