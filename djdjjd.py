#!/usr/bin/env python3
# adaptive_pid_nn.py
# Requires: numpy, matplotlib (pip install numpy matplotlib)

import numpy as np
import matplotlib.pyplot as plt

# ---------- Hyper-parameters ----------
SIM_STEPS     = 1000
DT            = 0.05          # discrete time-step (s)
PLANT_A       = 1.0           # ẏ = -a y³ + b u
PLANT_B       = 1.0
NN_HIDDEN     = 8
NN_LR         = 0.01          # learning-rate for the plant model
PID_LR        = 0.0005        # learning-rate for adaptive gains
Kp, Ki, Kd    = 1.0, 0.0, 0.0 # initial PID gains
np.random.seed(0)

# ---------- Tiny NN to approximate the plant ----------
class TinyMLP:
    def __init__(self, in_dim, hidden_dim):
        self.W1 = np.random.randn(hidden_dim, in_dim) * 0.5
        self.b1 = np.zeros((hidden_dim, 1))
        self.W2 = np.random.randn(1, hidden_dim) * 0.5
        self.b2 = np.zeros((1, 1))

    def _tanh(self, z):          return np.tanh(z)
    def _tanh_deriv(self, z):    return 1.0 - np.tanh(z)**2

    def forward(self, x):
        """x shape = (in_dim, 1)"""
        self.z1 = self.W1 @ x + self.b1
        self.a1 = self._tanh(self.z1)
        self.z2 = self.W2 @ self.a1 + self.b2
        return self.z2            # linear output

    def backward(self, x, target, lr=NN_LR):
        """Update weights using dJ/dW from (y_hat - target)^2 / 2"""
        y_hat = self.forward(x)
        error = y_hat - target      # shape (1,1)

        dW2 = error * self.a1.T
        db2 = error

        da1 = self.W2.T @ error
        dz1 = da1 * self._tanh_deriv(self.z1)
        dW1 = dz1 @ x.T
        db1 = dz1

        # Gradient step
        self.W2 -= lr * dW2
        self.b2 -= lr * db2
        self.W1 -= lr * dW1
        self.b1 -= lr * db1

        return error.item()        # scalar loss

    def grad_input(self, x):
        """∂y_hat / ∂x  (Jacobian w.r.t. 2-element input)"""
        self.forward(x)  # ensures z1, a1 cached
        # dy/dW2 = a1  ⇒ dy/d a1 = W2
        da1 = self.W2.T           # shape (hidden,1)
        dz1 = da1 * self._tanh_deriv(self.z1)   # shape (hidden,1)
        dinput = self.W1.T @ dz1                # (in_dim,1)
        return dinput.flatten()                 # (2,)

# ---------- PID Controller ----------
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        u = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        # Store parts for grad
        self._last_terms = (error, self.integral, derivative)
        return u

    def adapt_gains(self, grad_J_u, lr=PID_LR):
        """Gradient descent on J = ½ e² using ∂J/∂u = -e * dy/du"""
        e, I, D = self._last_terms
        # u = Kp*e + Ki*I + Kd*D  ⇒ ∂u/∂Kp = e, etc.
        self.Kp -= lr * grad_J_u * e
        self.Ki -= lr * grad_J_u * I
        self.Kd -= lr * grad_J_u * D

# ---------- Non-linear plant ----------
def plant(y, u):
    """Euler integrate ẏ = -a y³ + b u"""
    dydt = -PLANT_A * y**3 + PLANT_B * u
    return y + DT * dydt

# ---------- Dynamic set-point ----------
def setpoint(t):
    if t < 200:
        return 1.0
    elif t < 400:
        return -1.0
    else:
        return 1.5 * np.sin(0.02 * np.pi * t)

# ---------- Simulation ----------
nn = TinyMLP(in_dim=2, hidden_dim=NN_HIDDEN)
pid = PID(Kp, Ki, Kd)

y = 0.0
history = {"y":[], "r":[], "u":[], "Kp":[], "Ki":[], "Kd":[]}

for k in range(SIM_STEPS):
    r = setpoint(k)
    e = r - y
    u = pid.update(e, DT)

    # Step true plant
    y_next = plant(y, u)

    # --- Train NN on (y, u) → y_next ---
    x = np.array([[y],[u]])            # column vector
    nn.backward(x, np.array([[y_next]]))

    # --- Adapt PID gains using NN gradient ---
    # ∂J/∂u = -e * ∂y/∂u   (since J = ½ e² and y influences next error)
    dy_du = nn.grad_input(x)[1]        # derivative wrt u (index 1)
    grad_J_u = -e * dy_du
    pid.adapt_gains(grad_J_u)

    # Log
    history["y"].append(y)
    history["r"].append(r)
    history["u"].append(u)
    history["Kp"].append(pid.Kp)
    history["Ki"].append(pid.Ki)
    history["Kd"].append(pid.Kd)

    y = y_next  # advance

# ---------- Plot results ----------
t = np.arange(SIM_STEPS) * DT

plt.figure()
plt.plot(t, history["r"], label="Set-point r(t)")
plt.plot(t, history["y"], label="Plant output y(t)")
plt.xlabel("time (s)"); plt.ylabel("value"); plt.legend(); plt.title("Adaptive NN-PID Tracking")

plt.figure()
plt.plot(t, history["Kp"], label="Kp")
plt.plot(t, history["Ki"], label="Ki")
plt.plot(t, history["Kd"], label="Kd")
plt.xlabel("time (s)"); plt.ylabel("gain value"); plt.legend(); plt.title("PID Gains Adapting")

plt.show()