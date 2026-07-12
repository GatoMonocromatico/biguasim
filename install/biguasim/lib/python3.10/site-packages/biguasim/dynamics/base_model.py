import torch

import numpy as np

from abc import ABC, abstractmethod
from torch import Tensor
from operator import itemgetter

from biguasim.dynamics.utils import BatchedParams



    
def parse_state(state):
    if isinstance(state, dict):
    # if 'DynamicsSensor' in state.keys():
        return [state['DynamicsSensor']]
    state.pop('t')
    return list(map(itemgetter('DynamicsSensor'), itemgetter(*state.keys())(state)))


def extract_dynamics(sensor_list):
    out = []
    for entry in sensor_list:
        ds = entry.get("DynamicsSensor")
        if ds is None:
            continue
        # Works for list / numpy / tensor
        out.append(torch.as_tensor(ds).double())
    return out

class VehicleModel(ABC):
    def __init__(self, batch_size : int, params : dict, device : str, control_abstraction : str) -> None:  
       self.batch_size = batch_size                                              
       self.batched_params = BatchedParams(batch_size, params, device)           
       self.device = device                                                      
       self.control_abstraction = control_abstraction                            
                                                                          
       self.idxs = Tensor(range(batch_size)).int()       
       self.dt = 0
       

    @staticmethod
    def map_states(func):
        def wrapper(self, state: list, control: list, dt: int):
            # time step update
            self.dt = dt - self.dt

            # Extract dynamics -> list[Tensor]
            state = extract_dynamics(state)
            batch = len(state)

            if batch == 0:
                raise RuntimeError("No DynamicsSensor found in state")

            # Stack once: (B, N)
            dynamics = torch.stack(state, dim=0).to(self.device).double()

            # Controls: (B, C)
            control = torch.as_tensor(control, device=self.device).double()

            # Slice everything at once (NO Python loop)
            s = {
                'x': dynamics[:, 6:9],      # position
                'v': dynamics[:, 3:6],      # velocity
                'q': dynamics[:, 15:19],    # quaternion
                'w': dynamics[:, 12:15],    # angular velocity
            }

            c = {
                'cmd_ctrl': control
            }

            return func(self, s, c, dt)

        return wrapper

    @staticmethod
    def pack_state(state, batch_size, device):                                                                             
        """                                                                                                                
        Convert a state dict to Quadrotor's private internal vector 
        representation.                                        
        """                                                                                                                
        s = torch.zeros(batch_size, 13, device=device).double()   
        s[..., 0:3] = state['x']  # inertial position                                                                      
        s[..., 3:6] = state['v']  # inertial velocity                                                                      
        s[..., 6:10] = state['q']  # orientation                                                                           
        s[..., 10:13] = state['w']  # body rates                                                                           
                                                                                                                        
        return s    

    @staticmethod
    def unpack_state(s, idxs, batch_size):                                             
        """                                                                            
        Convert Quadrotor's private internal vector representation to a 
        state dict.    
        x = inertial position                                                          
        v = inertial velocity                                                          
        q = orientation                                                                
        w = body rates                                                                 
        """                                                                            
        device = s.device                                                              
        state = {                                                                      
            'x': torch.full((batch_size, 3), float("nan"), 
                            device=device).double(),   
            'v': torch.full((batch_size, 3), float("nan"), 
                            device=device).double(),   
            'q': torch.full((batch_size, 4), float("nan"), 
                            device=device).double(),   
            'w': torch.full((batch_size, 3), float("nan"), 
                            device=device).double(),   
            }                                                                          
        state['q'][..., -1] = 1  # make sure we're returning a valid quaternion        
        state['x'][idxs] = s[:, 0:3]                                                   
        state['v'][idxs] = s[:, 3:6]                                                   
        state['q'][idxs] = s[:, 6:10]                                                  
        state['w'][idxs] = s[:, 10:13]                                                 
        return state           
    
    @abstractmethod
    def _build_params(self, params : dict) -> None:
        pass


    @abstractmethod
    def _compute_external_forces(self, *args, **kwargs) -> Tensor:
        pass


    @abstractmethod 
    def _compute_body_wrench(self, *args, **kwargs) -> tuple:
        pass

    @abstractmethod 
    def _s_dot_fn(self, s : Tensor, cmd_ctrl : Tensor) -> Tensor:
        pass

    @abstractmethod 
    def get_cmd_motor_speeds(self, state : dict, control : dict) -> tuple[Tensor, Tensor]:  
        pass


    @map_states
    def step(self, state : dict, control : dict, dt : int):
        """
        Integrate dynamics forward from state given constant control for time t_step.
        Params:
            - state: dictionary containing keys ['x', 'v', 'q', 'w', rotor_speeds'], and values which
              are pytorch tensors with dtype double and which have a batch dimension
            - control: dictionary with keys depending on the chosen control mode. Values are torch 
            tensors, again with dtype double and with a batch dimension equal to the number of agents.
        """
        if self.control_abstraction == 'accel':
            return control['cmd_ctrl'][self.idxs].cpu().tolist()
        
        cmd_ctrl = self.get_cmd_motor_speeds(state, control)
        cmd_ctrl = torch.clip(
            cmd_ctrl,
            self.batched_params.rotor_speed_min[self.idxs],
            self.batched_params.rotor_speed_max[self.idxs]
        )

        s = self.pack_state(state, self.batch_size, self.device)
        s_dot = self._s_dot_fn(s, cmd_ctrl)
        v_dot = torch.zeros_like(state["v"])
        w_dot = torch.zeros_like(state["w"])
        v_dot[self.idxs] = s_dot[..., 3:6].double()
        w_dot[self.idxs] = s_dot[..., 10:13].double()

        state = self.unpack_state(s, self.idxs, self.batch_size)
        state['q'][self.idxs] = state['q'][self.idxs] / \
                                torch.norm(state['q'][self.idxs], dim=-1).unsqueeze(-1)

        # return Transform.convert_NED_to_NWU(state['q'][self.idxs], v_dot, w_dot).cpu().tolist()    
        return torch.cat([v_dot, w_dot], dim=1).cpu().tolist()