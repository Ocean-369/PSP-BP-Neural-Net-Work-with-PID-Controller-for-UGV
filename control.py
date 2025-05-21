import numpy as np

class PSOBPNNPIDController:
    def __init__(self):
        self.xite = 0.50
        self.alfa = 0.05
        self.IN, self.H, self.OUT = 4, 5, 3
        self.M = np.array([9.9, 9.8, 9.4])
        self.reset_weights()
        self.reset_state()

    def reset_weights(self):
        self.wi = np.array([
            [-4.7730, 5.0000, 4.8238, -4.7085],
            [4.9337, 4.8659, -4.9651, -4.8618],
            [4.9470, 4.9485, -4.6965, 4.8327],
            [5.0000, 4.9508, 4.5696, 4.9154],
            [4.6047, -4.7804, 5.0000, -4.7332],
        ])
        self.wo = np.array([
            [4.9438, -4.6610, 4.9293, 4.7416, -4.9323],
            [5.0000, 4.9414, 4.0693, -4.7460, 5.0000],
            [-4.9147, 4.8119, 4.9292, 4.9434, 4.9398],
        ])
        self.wi_1 = self.wi.copy()
        self.wi_2 = self.wi.copy()
        self.wo_1 = self.wo.copy()
        self.wo_2 = self.wo.copy()

    def reset_state(self):
        self.y_1 = self.y_2 = self.error_1 = self.error_2 = self.du_1 = 0
        self.u_1 = self.u_2 = 0
        self.kp, self.ki, self.kd = [], [], []
        self.u, self.du, self.yout, self.error = [], [], [], []

    def run(self, X_data, y_data):
        for xi, (ref, yk) in zip(X_data, y_data):
            ek = ref - yk
            epid = np.array([xi[0], xi[1], xi[2]])

            net2 = self.wi @ xi
            Oh = (np.exp(net2) - np.exp(-net2)) / (np.exp(net2) + np.exp(-net2))
            net3 = self.wo @ Oh
            K = np.exp(net3) / (np.exp(net3) + np.exp(-net3))
            kpid = self.M * K
            dukt = kpid @ epid
            uk = self.u_1 + dukt
            uk = np.clip(uk, -10, 10)

            dyu = np.sign((yk - self.y_1) / (dukt - self.du_1 + 1e-5))
            dK = 1 / (np.exp(net3) + np.exp(-net3))
            delta3 = np.array([ek * dyu * epid[i] * dK[i] for i in range(self.OUT)])
            d_wo = self.xite * np.outer(delta3, Oh) + self.alfa * (self.wo_1 - self.wo_2)
            self.wo = self.wo_1 + d_wo + self.alfa * (self.wo_1 - self.wo_2)

            dO = 4 / (np.exp(net2) + np.exp(-net2))**2
            sigma = delta3 @ self.wo
            delta2 = dO * sigma
            d_wi = self.xite * np.outer(delta2, xi)
            self.wi = self.wi_1 + d_wi + self.alfa * (self.wi_1 - self.wi_2)

            self.wi_2, self.wi_1 = self.wi_1, self.wi.copy()
            self.wo_2, self.wo_1 = self.wo_1, self.wo.copy()
            self.du_1 = dukt
            self.u_2, self.u_1 = self.u_1, uk
            self.y_2, self.y_1 = self.y_1, yk
            self.error_2, self.error_1 = self.error_1, ek

            self.kp.append(kpid[0])
            self.ki.append(kpid[1])
            self.kd.append(kpid[2])
            self.u.append(uk)
            self.du.append(dukt)
            self.yout.append(yk)
            self.error.append(ek)
