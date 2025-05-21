import numpy as np

def generate_pid_data(steps=200, ts=0.01):
    num = [0, 0.0784, 0.0776]
    den = [1.0, -1.9025, 0.9048]
    y_1 = y_2 = e_prev = u_1 = u_2 = 0
    setpoint = 1.0
    X_data, y_data = [], []
    for _ in range(steps):
        y = -den[1]*y_1 - den[2]*y_2 + num[1]*u_1 + num[2]*u_2
        e = setpoint - y
        de = e - e_prev
        dde = e - 2*e_prev
        xii = np.array([de, e, dde, 1.0])
        xi = xii / np.linalg.norm(xii)
        X_data.append(xi)
        y_data.append((setpoint, y))
        e_prev, y_2, y_1 = e, y_1, y
        u_2, u_1 = u_1, u_1  # 输入暂不更新
    time = np.arange(steps) * ts
    return np.array(X_data), y_data, time