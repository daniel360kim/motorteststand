import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt

# Simulated nonlinear plant (ground truth)
def plant_model(u_prev, y_prev):
    """Nonlinear second-order plant"""
    return 0.7 * y_prev[-1] - 0.2 * y_prev[-2] + 0.1 * np.tanh(u_prev[-1]) + 0.05 * np.random.randn()

# MLP for system identification
class PlantMLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 32),
            nn.Tanh(),
            nn.Linear(32, 1)
        )
    
    def forward(self, x):
        return self.net(x)

# Sim setup
n_steps = 300
setpoint = 1.0
u_prev = [0.0, 0.0]
y_prev = [0.0, 0.0]
mlp = PlantMLP()
optimizer = torch.optim.Adam(mlp.parameters(), lr=0.01)
loss_fn = nn.MSELoss()

# PID gains (learned)
Kp = torch.tensor(0.5, requires_grad=True)
Ki = torch.tensor(0.1, requires_grad=True)
Kd = torch.tensor(0.01, requires_grad=True)
pid_params = [Kp, Ki, Kd]
pid_optimizer = torch.optim.SGD([Kp, Ki, Kd], lr=0.01)

# Storage
us, ys, es, y_preds = [], [], [], []
int_e = 0
last_e = 0

for t in range(n_steps):
    # True plant output
    y = plant_model(u_prev, y_prev)
    ys.append(y)

    # Error
    e = setpoint - y
    es.append(e)
    int_e += e
    der_e = e - last_e
    last_e = e

    # Control input from current PID params
    u = Kp.item() * e + Ki.item() * int_e + Kd.item() * der_e
    u = np.clip(u, -1.0, 1.0)
    us.append(u)

    # Train the plant model online
    x_mlp = torch.tensor([u_prev[-1], u_prev[-2], y_prev[-1], y_prev[-2]], dtype=torch.float32)
    y_target = torch.tensor([y], dtype=torch.float32)
    y_pred = mlp(x_mlp)
    y_preds.append(y_pred.item())

    plant_loss = loss_fn(y_pred, y_target)
    optimizer.zero_grad()
    plant_loss.backward()
    optimizer.step()

    # Estimate how good the current PID is using predicted output
    # Use same features but with current error terms
    e_torch = torch.tensor([e], dtype=torch.float32)
    int_e_torch = torch.tensor([int_e], dtype=torch.float32)
    der_e_torch = torch.tensor([der_e], dtype=torch.float32)
    
    pid_u = Kp * e_torch + Ki * int_e_torch + Kd * der_e_torch
    x_sim = torch.stack([pid_u, torch.tensor([u_prev[-1]]), torch.tensor([y_prev[-1]]), torch.tensor([y_prev[-2]])], dim=-1).squeeze()

    y_sim = mlp(x_sim.detach())  # don't backprop through model
    sim_loss = (setpoint - y_sim).pow(2)

    pid_optimizer.zero_grad()
    sim_loss.backward()
    pid_optimizer.step()

    # Update histories
    u_prev = [u_prev[-1], u]
    y_prev = [y_prev[-1], y]

print(f"Final PID Gains: Kp={Kp.item():.3f}, Ki={Ki.item():.3f}, Kd={Kd.item():.3f}")

# --- Plotting ---
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(ys, label="Plant Output")
plt.plot([setpoint]*n_steps, 'k--', label="Setpoint")
plt.plot(y_preds, label="NN Prediction")
plt.legend()
plt.title("Plant Output and MLP Prediction")

plt.subplot(2, 1, 2)
plt.plot(us, label="Control Input (u)")
plt.xlabel("Time step")
plt.legend()
plt.tight_layout()
plt.show()