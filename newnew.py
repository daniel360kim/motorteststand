"""
adaptive_pid_nn.py
Run:  python adaptive_pid_nn.py
"""

import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------
# 1.  Plant (ground truth) and helper
# --------------------------------------------------
A, B = 0.9, 0.5                    # y[t+1] = A*y[t] + B*u[t]
def plant(y, u):
    return A * y + B * u

# --------------------------------------------------
# 2.  Tiny 2-layer MLP to learn y[t+1] from [u[t],u[t-1],y[t],y[t-1]]
# --------------------------------------------------
class MLP:
    def __init__(self, in_size, hidden, out_size, lr=1e-2):
        rng = np.random.default_rng(0)
        self.w1 = rng.normal(0, 0.1, (hidden, in_size))
        self.b1 = np.zeros((hidden, 1))
        self.w2 = rng.normal(0, 0.1, (out_size, hidden))
        self.b2 = np.zeros((out_size, 1))
        self.lr = lr

    @staticmethod
    def _sigmoid(z):
        return 1 / (1 + np.exp(-z))

    def forward(self, x):
        self.z1  = self.w1 @ x + self.b1
        self.a1  = self._sigmoid(self.z1)
        self.z2  = self.w2 @ self.a1 + self.b2
        self.y   = self.z2                 # linear output
        return self.y

    # one-step gradient descent
    def backward_and_update(self, x, y_true):
        # dJ/dy = y_pred - y_true   (½ L2 loss)
        dL_dy   = (self.y - y_true)
        dL_dw2  = dL_dy @ self.a1.T
        dL_db2  = dL_dy
        dL_da1  = self.w2.T @ dL_dy
        dL_dz1  = dL_da1 * self.a1 * (1 - self.a1)
        dL_dw1  = dL_dz1 @ x.T
        dL_db1  = dL_dz1

        for w, g in zip([self.w1, self.w2, self.b1, self.b2],
                        [dL_dw1, dL_dw2, dL_db1, dL_db2]):
            w -= self.lr * g

    # scalar ∂y/∂u[t]
    def dym_du(self):
        # chain rule through the two layers (only u[t] is x[0])
        sig_prime = self.a1 * (1 - self.a1)
        col_u_t   = self.w1[:, 0]          # weight column for u[t]
        row_out   = self.w2[0, :]          # single-output row
        return float((row_out * sig_prime.flatten() * col_u_t).sum())

# --------------------------------------------------
# 3.  PID with momentum & adaptive step
# --------------------------------------------------
class AdaptivePID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, beta=0.9):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.int_e = 0.0
        self.prev_e = 0.0
        self.d_filt = 0.0
        self.v = np.zeros(3)               # momentum buffer
        self.beta = beta

    def update(self, e, dt, d_alpha=0.1):
        self.int_e += e * dt
        raw_d = (e - self.prev_e) / dt if dt else 0.0
        self.d_filt = (1 - d_alpha) * self.d_filt + d_alpha * raw_d
        self.prev_e = e
        return (self.Kp * e +
                self.Ki * self.int_e +
                self.Kd * self.d_filt)

    def adapt(self, dym_du, e, lr=1e-3):
        g_common = -e * dym_du
        grads = np.array([g_common * e,
                          g_common * self.int_e,
                          g_common * self.d_filt])
        self.v = self.beta * self.v + (1 - self.beta) * grads
        step = lr * self.v
        self.Kp -= step[0]
        self.Ki -= step[1]
        self.Kd -= step[2]
        # keep gains non-negative
        self.Kp = max(self.Kp, 0)
        self.Ki = max(self.Ki, 0)
        self.Kd = max(self.Kd, 0)

# --------------------------------------------------
# 4.  Simulation
# --------------------------------------------------
dt       = 0.05
T_final  = 20
t_axis   = np.arange(0, T_final, dt)

# buffers for NN input: [u[t], u[t-1], y[t], y[t-1]]
u_buf = [0.0, 0.0]
y_buf = [0.0, 0.0]

plant_net = MLP(in_size=4, hidden=6, out_size=1, lr=5e-3)
pid       = AdaptivePID(Kp=0.5, Ki=0.0, Kd=0.0)
lr_pid    = 5e-4

setpoint  = lambda t: 1.0 if t > 1.0 else 0.0   # step at t=1 s

# logs
ys, ys_hat, us, es, Kps, Kis, Kds = [], [], [], [], [], [], []

y = 0.0
for t in t_axis:
    r   = setpoint(t)
    e   = r - y
    u   = pid.update(e, dt)
    y_next = plant(y, u)              # ground-truth plant

    # NN forward + back-prop
    x = np.array([[u_buf[0], u_buf[1], y_buf[0], y_buf[1]]]).T
    y_hat = plant_net.forward(x)
    plant_net.backward_and_update(x, np.array([[y_next]]))
    dym_du = plant_net.dym_du()

    # PID adaptation
    pid.adapt(dym_du, e, lr_pid)

    # roll buffers
    u_buf = [u, u_buf[0]]
    y_buf = [y_next, y_buf[0]]
    y = y_next

    # log everything
    ys.append(y)
    ys_hat.append(float(y_hat))
    us.append(u)
    es.append(e)
    Kps.append(pid.Kp)
    Kis.append(pid.Ki)
    Kds.append(pid.Kd)

# --------------------------------------------------
# 5.  Plots
# --------------------------------------------------
plt.figure(figsize=(9, 5))
plt.plot(t_axis, ys, label="Plant output y")
plt.plot(t_axis, [setpoint(t) for t in t_axis], "--", label="Set-point r")
plt.xlabel("time [s]"); plt.ylabel("output"); plt.title("Adaptive PID tracking")
plt.legend(); plt.grid(True)

plt.figure(figsize=(9, 4))
plt.plot(t_axis, Kps, label="Kp")
plt.plot(t_axis, Kis, label="Ki")
plt.plot(t_axis, Kds, label="Kd")
plt.xlabel("time [s]"); plt.ylabel("gain value"); plt.title("Gain adaptation")
plt.legend(); plt.grid(True)

plt.tight_layout()
plt.show()