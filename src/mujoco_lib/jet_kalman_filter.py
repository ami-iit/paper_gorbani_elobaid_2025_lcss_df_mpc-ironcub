import casadi as cs
import numpy as np

class SecondOrderJetModel:
    def __init__(self, R, Q, P, dt):
        self.coeffs = [-4.64730485e-01,
                  -8.13171858e+00,
                  -6.19539230e+00,
                  6.61113140e-01,
                  1.67673231e+00,
                  -4.83287064e-01,
                  8.77996617e+00,
                  -1.01096376e+00,
                  -5.86442286e-01,
                  5.19093322e-01,
                  -4.23782666e-01,
                  -1.45705257e+00,
                  -7.83052261e-03]
        self.mean_thrust = 108.309
        self.std_thrust = 65.793
        self.mean_throttle = 47.333
        self.std_throttle = 31.483
        self.R = R
        self.Q = Q
        self.P = P
        self.dt = dt
        self.f = self.get_cs_f_fun()
        self.A, self.B = self.get_jacobians()

    def get_cs_f_fun(self):
        x = cs.MX.sym("x", 2)
        T = x[0]
        T_dot = x[1]
        u = cs.MX.sym("throttle")
        thrustStd = (T - self.mean_thrust) / self.std_thrust
        thrustDotStd = T_dot / self.std_thrust
        throttleStd = (u - self.mean_throttle) / self.std_throttle
        f = self.coeffs[0] + self.coeffs[1] * thrustStd + self.coeffs[2] * thrustDotStd + self.coeffs[3] * thrustStd * thrustDotStd + self.coeffs[4] * thrustStd**2 + self.coeffs[5] * thrustDotStd**2
        g = self.coeffs[6] + self.coeffs[7] * thrustStd + self.coeffs[8] * thrustDotStd + self.coeffs[9] * thrustStd * thrustDotStd + self.coeffs[10] * thrustStd**2 + self.coeffs[11] * thrustDotStd**2
        v = throttleStd + self.coeffs[12] * throttleStd**2
        T_dot_dot = f + g * v
        T_dot = T_dot + T_dot_dot * self.std_thrust * self.dt
        T = T + T_dot * self.dt
        x_updated = cs.vertcat(T, T_dot)
        return cs.Function("f", [x, u], [x_updated])

    def get_jacobians(self):
        x = cs.MX.sym("x", 2)
        u = cs.MX.sym("throttle")
        A = cs.jacobian(self.f(x, u), x)
        B = cs.jacobian(self.f(x, u), u)
        A = cs.Function("A", [x, u], [A])
        B = cs.Function("B", [x, u], [B])
        return A, B
    
    def update(self, x, u, z):
        H = cs.DM.eye(2)
        x = self.f(x, u)
        self.P = self.A(x, u) @ self.P @ self.A(x, u).T + self.Q
        err = z - H @ x
        S = H @ self.P @ H.T + self.R # R = 0
        K = self.P @ H.T @ cs.inv(S)
        x = x + K @ err
        self.P = (cs.DM.eye(2) - K @ H) @ self.P
        return x

class EKFJetsTotal:
    def __init__(self, R, Q, P, dt):
        self._n_jets = 4
        self._EKF_jets = [SecondOrderJetModel(R, Q, P, dt) for _ in range(self._n_jets)]

    def update(self, T, TDot, u, TMeas, TDotMeas):
        T_new = np.zeros(self._n_jets)
        TDot_new = np.zeros(self._n_jets)
        for i in range(self._n_jets):
            z = cs.vertcat(TMeas[i], TDotMeas[i])
            x = cs.vertcat(T[i], TDot[i])
            x = self._EKF_jets[i].update(x, u[i], z)
            T_new[i] = x[0]
            TDot_new[i] = x[1]
        return T_new, TDot_new
        
