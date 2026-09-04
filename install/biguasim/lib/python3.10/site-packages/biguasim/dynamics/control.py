import torch
from torch import Tensor

class BatchedPIDController:
    def __init__(self, Kp, Ki, Kd, batch_shape, device="cpu"):
        # Scalars or tensors of shape (B,)
        self.Kp = torch.as_tensor(Kp, device=device)
        self.Ki = torch.as_tensor(Ki, device=device)
        self.Kd = torch.as_tensor(Kd, device=device)

        # States (initialized to zeros, will be shaped by batch_size)
        self.previous_error = torch.zeros(batch_shape, device=device)
        self.integral = torch.zeros(batch_shape, device=device)
        self.setpoints = torch.zeros(batch_shape, device=device)

    def update_setpoints(self, new_setpoints : Tensor, mask = None):
        """Update setpoints for the batch (shape: batch_size)."""
        self.setpoints = torch.as_tensor(new_setpoints, device=self.setpoints.device)
        if isinstance(mask, Tensor):
            self.previous_error = self.previous_error * mask
            self.integral = self.integral * mask

    def compute(self, process_variables : Tensor , dt : float, idx):
        # error = (B,)
        error = self.setpoints[idx] - process_variables[idx]

        # Proportional
        P_out = self.Kp * error[idx]

        # Integral
        self.integral += error[idx] * dt
        I_out = self.Ki * self.integral[idx]

        # Derivative
        derivative = (error[idx] - self.previous_error[idx]) / dt
        D_out = self.Kd * derivative[idx]

        # Update states
        self.previous_error = error.clone()

        return P_out + I_out + D_out
        # return P_out + D_out
    