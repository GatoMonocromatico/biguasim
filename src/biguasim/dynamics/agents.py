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
    
#class of the competition drone
class HolybroX500(uav.QuadCopterX):
    _params = {
        # Inertial properties (official PX4 x500 model, mass in kg)
        'mass': 2.0,            # kg, all-up weight of the Holybro X500 V2

        'rho' : 1225,       # Air density

        # Inertia tensor (kg*m^2) from the PX4 x500_base model
        'I' : np.diag([0.021666666, 0.021666666, 0.04]),

        # Geometric properties, all vectors relative to center of mass.
        # PX4 x500 rotors sit at +-0.174 m on both body axes -> in-plane
        # distance to each rotor is 0.174*sqrt(2) ~= 0.246 m.
        'd' : 0.246,             # Distance from CoM to each rotor, m

        'rotor_pos': {          # location of each rotor in meters
            'r1': 0.246 * np.array([0.70710678118, 0.70710678118, 0]),       # Rotor 1 position
            'r2': 0.246 * np.array([0.70710678118, -0.70710678118, 0]),       # Rotor 2 position
            'r3': 0.246 * np.array([-0.70710678118, -0.70710678118, 0]),      # Rotor 3 position
            'r4': 0.246 * np.array([-0.70710678118, 0.70710678118, 0]),      # Rotor 4 position
        },

        # Rotor coefficients from the PX4 x500 motor plugin:
        #   thrust  = k_eta * omega^2  (motorConstant = 8.54858e-06)
        #   torque  = k_m   * omega^2  (k_m = momentConstant * motorConstant = 0.016 * 8.54858e-06)
        'k_eta' : 8.54858e-6,
        'k_m' : 1.36777e-7,

        'rotor_directions': np.array([1, -1, 1, -1]),  # Rotor spin directions (+1 for CW, -1 for CCW)
        'rotor_speed_min': 0,      # minimum rotor speed, rad/s
        'rotor_speed_max': 1000.0, # maximum rotor speed, rad/s (PX4 maxRotVelocity)

        # Scale the yaw moment by Izz so it is consistent with the roll/pitch moments,
        # and decouple the cmd_pos_yaw yaw loop from the attitude stiffness. Both are
        # required for this low-inertia airframe to keep the yaw modes stable.
        'scale_yaw_by_inertia': True,
        'pos_yaw_decoupled': True,
        # cmd_pos_yaw heading governor: slew large yaw commands through small, stable
        # yaw errors. 0.2 rad keeps rotate-in-place robust for any target yaw (incl 180).
        'yaw_slew_max': 0.2,

        # Frame aerodynamic properties
        'c_Dx': 0.1,            # parasitic drag coefficient in body x-axis, N/(m/s)^2
        'c_Dy': 0.1,            # parasitic drag coefficient in body y-axis, N/(m/s)^2
        'c_Dz': 0.15,           # parasitic drag coefficient in body z-axis, N/(m/s)^2

        # Lower level controller properties (for higher level control abstractions).
        # Tuned for the X500's low inertia / high thrust-to-weight: the attitude loop
        # needs strong damping (kd_att) to stay stable at the 30 Hz control rate.
        'k_v': 2.0,             # The *world* velocity P gain (for cmd_vel)
        'kp_att': 10.0,         # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'kd_att': 6.0,          # The attitude D gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_yaw': 1.0,          # The yaw P gain (for cmd_vel_yaw / cmd_pos_yaw)
        'kd_yaw': 2.0,          # The yaw D gain (yaw moment is Izz-scaled, so ~O(1) gains)

        'kp_pos': 0.4,          # The position P gain (for cmd_pos_yaw)
        'kd_pos': 1.6,          # The position D gain (for cmd_pos_yaw, well damped)

    }

    _scheme = 1
    
    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):
        super().__init__(
                        batch_size, 
                        params= params or HolybroX500._params,
                        device=device,
                        control_abstraction=control_abstraction)

        self._params = params or HolybroX500._params

    @property
    def params(self) -> dict: 
        return self._params
    
#class of the competition drone: Holybro Kopis X8 Cinelifter 5" (Caged).
#The real airframe is an X8 coaxial octo (4 arms, top+bottom rotor each), but
#only the FOUR TOP rotors are modelled here, so it is a plain QuadCopterX.
#Mass and inertia still describe the complete physical drone -- the bottom
#motors and the cage are on board, they just do not produce thrust in the model.
class KopisX8(uav.QuadCopterX):
    _params = {
        # Inertial properties.
        # 1.2 kg full kit (Holybro) + ~0.8 kg 6S 5000 mAh LiPo = ~2.0 kg AUW,
        # no cinema payload. The frame is rated for 1.5 kg of payload on top.
        'mass': 2.0,            # kg, all-up weight without payload

        'rho' : 1225,       # Air density

        # Inertia tensor (kg*m^2), estimated from the published geometry:
        #   8 x 35.1 g Velox V2207 V2 at r = 0.125 m, 570 g caged carbon frame
        #   treated as a 0.315 x 0.315 m plate, and ~1.15 kg of battery +
        #   electronics as a compact central box. Izz/Ixx ~ 1.8, typical.
        'I' : np.diag([0.0084, 0.0084, 0.0150]),

        # Geometric properties, all vectors relative to center of mass.
        # Holybro gives the caged footprint as 315 x 315 mm; with a ~140 mm
        # guard ring around each 5" (127 mm) prop that puts the motors at
        # ~0.125 m from the CoM, i.e. a 250 mm wheelbase.
        'd' : 0.125,             # Distance from CoM to each rotor, m

        'rotor_pos': {          # location of each rotor in meters (top layer only)
            'r1': 0.125 * np.array([0.70710678118, 0.70710678118, 0]),       # Rotor 1 position
            'r2': 0.125 * np.array([0.70710678118, -0.70710678118, 0]),       # Rotor 2 position
            'r3': 0.125 * np.array([-0.70710678118, -0.70710678118, 0]),      # Rotor 3 position
            'r4': 0.125 * np.array([-0.70710678118, 0.70710678118, 0]),      # Rotor 4 position
        },

        # Rotor coefficients for the T-Motor Velox V2207 V2 1750KV turning a
        # Gemfan Hulkie 5055S-3 on 6S. Anchored on T-Motor's bench figures for
        # this motor: 1681 g (16.5 N) at 22.3 V, 36.6 A, 816 W.
        #   thrust = k_eta * omega^2 -> 16.5 N at the 2950 rad/s cap
        #   torque = k_m   * omega^2, k_m = 0.0125 * k_eta (5" prop moment ratio)
        # Cross-check: k_m * omega_max^3 = 610 W of shaft power, i.e. 75% motor
        # + ESC efficiency against the published 816 W input -- consistent.
        # Four rotors give 66.1 N against 19.6 N of weight -> T/W ~ 3.4, and
        # hover sits at ~1606 rad/s (54% of max).
        'k_eta' : 1.90e-6,
        'k_m' : 2.38e-8,

        'rotor_directions': np.array([1, -1, 1, -1]),  # Rotor spin directions (+1 for CW, -1 for CCW)
        'rotor_speed_min': 0,      # minimum rotor speed, rad/s
        # 1750KV * 22.3 V = 39.0k RPM unloaded; a 5" prop pulls that down to
        # ~72%, i.e. ~28.2k RPM. For the 1950KV variant of the same motor only
        # this line changes (~3200 rad/s) -- k_eta belongs to the prop, not the
        # motor -- along with the matching caps in agents.py and vehicle.py.
        'rotor_speed_max': 2950.0, # maximum rotor speed, rad/s (~28.2k RPM on 6S)

        # Same two fixes the X500 needed: Izz here is even smaller (0.0150), so
        # the raw yaw moment over-drives yaw unless it is scaled by Izz, and the
        # cmd_pos_yaw yaw loop must be decoupled from the attitude stiffness.
        'scale_yaw_by_inertia': True,
        'pos_yaw_decoupled': True,
        # cmd_pos_yaw heading governor: slew large yaw commands through small,
        # stable yaw errors.
        'yaw_slew_max': 0.4,

        # Frame aerodynamic properties. Smaller span than the X500 but the cage
        # and the eight motor mounts add side area, and the caged top/bottom
        # make the vertical axis the bluffest.
        'c_Dx': 0.1,            # parasitic drag coefficient in body x-axis, N/(m/s)^2
        'c_Dy': 0.1,            # parasitic drag coefficient in body y-axis, N/(m/s)^2
        'c_Dz': 0.18,           # parasitic drag coefficient in body z-axis, N/(m/s)^2

        # Lower level controller properties (for higher level control abstractions).
        # Tuned against the Competition/CompetionMap engine at 30 Hz (2026-09-03),
        # measured with tests/tune_kopisx8.py --engine. Step-response score went
        # from 52.6 to 6.8; every maneuver now settles with no overshoot:
        #   climb 1 m/s  2.13 s      fwd 2 m/s  1.00 s  (was 25% overshoot, never settled)
        #   yaw 90 deg   1.03 s      goto x=5   1.77 s  (was 10.9 s)
        #
        # kp_att above ~50 buys nothing (the score plateaus at 7.44 all the way to
        # kp_att=100), so this is the least gain that reaches full performance --
        # the rest is margin against noise, disturbance and payload.
        #
        # k_v stays at 2.0 on purpose. Raising it speeds the climb up (2.13 -> 1.17 s)
        # but wrecks horizontal tracking (fwd overshoot 0% -> 16-24%): vertical error
        # becomes thrust directly, while horizontal error has to go through the
        # attitude loop, and a faster k_v destroys the timescale separation between
        # them. Splitting it into k_vxy / k_vz the way the UUV models do would fix
        # the climb, but that means changing uav.QuadCopterX, shared with the
        # HolybroX500 and the DjiMatrice.
        'k_v': 2.0,             # The *world* velocity P gain (for cmd_vel)
        'kp_att': 50.0,         # The attitude P gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)
        'kd_att': 10.0,         # The attitude D gain (for cmd_vel, cmd_vel_yaw and cmd_pos_yaw)

        'kp_yaw': 2.0,          # The yaw P gain (for cmd_vel_yaw / cmd_pos_yaw)
        'kd_yaw': 2.0,          # The yaw D gain (yaw moment is Izz-scaled, so ~O(1) gains)

        'kp_pos': 3.5,          # The position P gain (for cmd_pos_yaw)
        'kd_pos': 3.0,          # The position D gain (for cmd_pos_yaw, well damped)

    }

    _scheme = 1

    def __init__(self, batch_size=1, device='cpu', control_abstraction='cmd_motor_speeds', params= None):
        super().__init__(
                        batch_size,
                        params= params or KopisX8._params,
                        device=device,
                        control_abstraction=control_abstraction)

        self._params = params or KopisX8._params

    @property
    def params(self) -> dict:
        return self._params

class ModelsFactory:
    _types = {
        'BlueBoat' : BlueBoat,
        'BlueROV2' : BlueROV2,
        'BlueROVHeavy' : BlueROVHeavy,
        'DjiMatrice' : DjiMatrice,
        'TorpedoAUV' : TorpedoAUV,
        "HolybroX500": HolybroX500,
        "KopisX8" : KopisX8
}

    @classmethod
    def build_model(cls, agent_type : str):
        return cls._types[agent_type]


#começar por aqui
        



























