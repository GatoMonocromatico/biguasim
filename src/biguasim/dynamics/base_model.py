import torch
import numpy as np

from abc import ABC, abstractmethod
from torch import Tensor
from operator import itemgetter

from biguasim.dynamics.utils import BatchedParams


def _quat_to_rotmat(q: Tensor) -> Tensor:
    """Converte um quaternion [x, y, z, w] para uma matriz de rotação."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = torch.stack([
        1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w),
        2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w),
        2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)
    ], dim=-1).view(-1, 3, 3)
    return R


def parse_state(state):
    if isinstance(state, dict):
        return [state['DynamicsSensor']]
    state.pop('t')
    return list(map(itemgetter('DynamicsSensor'), itemgetter(*state.keys())(state)))


def extract_dynamics(sensor_list):
    out = []
    for entry in sensor_list:
        ds = entry.get("DynamicsSensor")
        if ds is None:
            continue
        out.append(torch.as_tensor(ds).double())
    return out


class VehicleModel(ABC):
    def __init__(self, batch_size: int, params: dict, device: str, control_abstraction: str) -> None:  
        self.batch_size = batch_size                                              
        self.batched_params = BatchedParams(batch_size, params, device)           
        self.device = device                                                      
        self.control_abstraction = control_abstraction                            
        self.idxs = Tensor(range(batch_size)).int()       
        self.dt = 0

    @staticmethod
    def map_states(func):
        def wrapper(self, state: list, control: list, dt: int):
            self.dt = dt - self.dt

            state = extract_dynamics(state)
            batch = len(state)

            if batch == 0:
                raise RuntimeError("No DynamicsSensor found in state")

            dynamics = torch.stack(state, dim=0).to(self.device).double()
            control = torch.as_tensor(control, device=self.device).double()

            # Extração de variáveis
            q = dynamics[:, 15:19]
            w_global = dynamics[:, 12:15]

            # Correção 1: Entrada (World -> Body)
            R_bw = _quat_to_rotmat(q).double()
            w_body = (R_bw.transpose(1, 2) @ w_global.unsqueeze(-1)).squeeze(-1)

            s = {
                'x': dynamics[:, 6:9],      
                'v': dynamics[:, 3:6],      
                'q': q,    
                'w': w_body,  # angular velocity convertida para BODY
            }

            c = {
                'cmd_ctrl': control
            }

            return func(self, s, c, dt)

        return wrapper

    @staticmethod
    def pack_state(state, batch_size, device):                                                                             
        s = torch.zeros(batch_size, 13, device=device).double()   
        s[..., 0:3] = state['x']  
        s[..., 3:6] = state['v']  
        s[..., 6:10] = state['q']  
        s[..., 10:13] = state['w']  
        return s    

    @staticmethod
    def unpack_state(s, idxs, batch_size):                                             
        device = s.device                                                              
        state = {                                                                      
            'x': torch.full((batch_size, 3), float("nan"), device=device).double(),   
            'v': torch.full((batch_size, 3), float("nan"), device=device).double(),   
            'q': torch.full((batch_size, 4), float("nan"), device=device).double(),   
            'w': torch.full((batch_size, 3), float("nan"), device=device).double(),   
        }                                                                          
        state['q'][..., -1] = 1  
        state['x'][idxs] = s[:, 0:3]                                                   
        state['v'][idxs] = s[:, 3:6]                                                   
        state['q'][idxs] = s[:, 6:10]                                                  
        state['w'][idxs] = s[:, 10:13]                                                 
        return state           
    
    @abstractmethod
    def _build_params(self, params: dict) -> None:
        pass

    @abstractmethod
    def _compute_external_forces(self, *args, **kwargs) -> Tensor:
        pass

    @abstractmethod 
    def _compute_body_wrench(self, *args, **kwargs) -> tuple:
        pass

    @abstractmethod 
    def _s_dot_fn(self, s: Tensor, cmd_ctrl: Tensor) -> Tensor:
        pass

    @abstractmethod 
    def get_cmd_motor_speeds(self, state: dict, control: dict) -> tuple[Tensor, Tensor]:  
        pass

    @map_states
    def step(self, state: dict, control: dict, dt: int):
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
        state['q'][self.idxs] = state['q'][self.idxs] / torch.norm(state['q'][self.idxs], dim=-1).unsqueeze(-1)

        # Correção 2: Saída angular (Body -> World)
        R_bw = _quat_to_rotmat(state['q'][self.idxs]).double()
        w_dot[self.idxs] = (R_bw @ w_dot[self.idxs].unsqueeze(-1)).squeeze(-1)

        return torch.cat([v_dot, w_dot], dim=1).cpu().tolist()
