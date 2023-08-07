"""
These classes describe the dynamics of different robots

"""

import numpy as np
import random
from scipy.integrate import odeint
from gym import spaces

# This class describes the dynamics for the planar task of autonomous underwater vehicles
class AUV():
    def __init__(self) -> None:
        self.Xu = 4
        self.Du = 0.4
        self.Yv = 5
        self.Dv = 0.4
        self.Nr = 1
        self.Dr = 0.2
        self.m11 = 5.2789
        self.m22 = 7.2789
        self.m33 = 0.0891

        self.dt = 0.05
        self.s0 = np.zeros(6)
        self.Nstates = 6
        self.umin = np.array([-1, -0.5]) # delta u
        self.umax = np.array([1, 0.5]) 

        low = np.array([-np.inf, -np.inf, -np.pi, -3, -2, - 60 / 180 * np.pi], dtype = np.float32)
        self.observation_space = spaces.Box(low, -low, dtype = np.float32)

    def reset(self):
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        psi = random.uniform(-np.pi, np.pi)
        u = random.uniform(-1, 1)
        v = 0
        r = random.uniform(-45 / 180 * np.pi, 45 / 180 * np.pi)
        self.s0 = np.array([x, y, psi, u, v, r])
        return self.s0

    def reset_state(self, s):
        self.s0 = s
        return self.s0

    def dynamics(self, f, t, input):
        x, y, psi, u, v, r = f
        input = np.array(input).reshape(2, 1)
        f = np.array(f).reshape(6, 1)
        ftem = np.zeros(6)
        Xu = self.Xu
        Du = self.Du
        Yv = self.Yv
        Dv = self.Dv
        Nr = self.Nr
        Dr = self.Dr
        m11 = self.m11
        m22 = self.m22
        m33 = self.m33
        c2 = np.cos(psi)
        s2 = np.sin(psi)
        C = np.zeros((6, 6))
        C[0, 3] = c2
        C[0, 4] = -s2
        C[1, 3] = s2
        C[1, 4] = c2
        C[2, 5] = 1
        C[3, 3] = (-Xu - Du * abs(u)) / m11
        C[3, 5] = m22 / m11 * v
        C[4, 4] = (-Yv - Dv * abs(v)) / m22
        C[4, 5] = -m11 / m22 * u
        C[5, 3] = -m22 / m33 * v
        C[5, 4] = m11 / m33 * u
        C[5, 5] = (-Nr - Dr * abs(r)) / m33
        D = np.zeros((6, 2))
        D[3, 0] = 1 / m11
        D[5, 1] = 1 / m33

        f = np.dot(C, f) + np.dot(D, input)
        ftem[0] = f[0]
        ftem[1] = f[1]
        ftem[2] = f[2]
        ftem[3] = f[3]
        ftem[4] = f[4]
        ftem[5] = f[5]
        return ftem

    def step(self, u):
        sn = odeint(self.dynamics, self.s0, [0, self.dt], args=(u, ))
        self.s0 = sn[-1, :]
        if self.s0[2] > np.pi:
            self.s0[2] = self.s0[2] - 2 * np.pi
        elif self.s0[2] < -np.pi:
            self.s0[2] = self.s0[2] + 2 * np.pi
        r = 0
        done = False
        return self.s0, r, done, {}
    
# This class describes the dynamics of the damping pendulum
class DP():
    def __init__(self) -> None:
        self.g = 9.8
        self.l = 1.0
        self.m = 1.0
        self.dt = 0.02
        self.s0 = np.zeros(2)
        self.Nstates = 2
        self.umin = np.array([-1]) # delta u
        self.umax = np.array([1])
        self.b = 1
        low = np.array([-np.pi, -8], dtype = np.float32)
        self.observation_space = spaces.Box(low, -low, dtype = np.float32)

    def reset(self):
        th0 = random.uniform(-0.1 * np.pi, 0.1 * np.pi)
        dth0 = random.uniform(-1, 1)
        self.s0 = np.array([th0, dth0])
        return self.s0

    def reset_state(self, s):
        self.s0 = s
        return self.s0

    def damping_pendulum(self, y, t, u):
        theta, dtheta = y
        f = np.asarray([dtheta, -self.g / self.l * np.sin(theta) - self.b * self.l * dtheta / self.m + np.cos(theta) * u / (self.m * self.l)])
        return f

    def step(self, u):
        u = np.array(u).reshape(1)
        sn = odeint(self.damping_pendulum, self.s0, [0, self.dt], args=(u[0], ))
        self.s0 = sn[-1, :]
        r = 0
        done = False

        return self.s0 ,r,done,{}

# This class describes the dynamics for the 3D task of autonomous underwater vehicles
class AUV_3D():
    def __init__(self) -> None:
        self.Xu = 4
        self.Du = 0.4
        self.Yv = 5
        self.Dv = 0.4
        self.Zw = 5
        self.Dw = 0.8
        self.Kp = 5
        self.Dp = 1
        self.Mq = 5
        self.Dq = 1
        self.Nr = 1
        self.Dr = 0.2
        self.m11 = 5.2789
        self.m22 = 7.2789
        self.m33 = 9
        self.m44 = 13
        self.m55 = 16
        self.m66 = 0.0891

        self.dt = 0.05
        self.s0 = np.zeros(12)
        self.Nstates = 12
        self.umin = np.array([-1, -0.1, -0.5]) # delta u
        self.umax = np.array([1, 0.1, 0.5])

        low = np.array([-np.inf, -np.inf, -np.inf, -np.pi, -np.pi / 2 * 0.99, -np.pi, -3, -2, -1, -60 / 180 * np.pi, -60 / 180 * np.pi, -60 / 180 * np.pi], dtype = np.float32)
        self.observation_space = spaces.Box(low, -low, dtype = np.float32)

    def reset(self):
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        z = random.uniform(-1, 1)
        phi = 0
        theta = 0
        psi = random.uniform(-np.pi, np.pi)
        u = random.uniform(-1, 1)
        v = 0
        w = 0
        p = 0
        q = 0
        r = random.uniform(-45 / 180 * np.pi, 45 / 180 * np.pi)
        self.s0 = np.array([x, y, z, phi, theta, psi, u, v, w, p, q, r])
        return self.s0

    def reset_state(self, s):
        self.s0 = s
        return self.s0

    def dynamics(self, f, t, input):
        x, y, z, phi, theta, psi, u, v, w, p, q, r = f
        input = np.array(input).reshape(3, 1)
        f = np.array(f).reshape(12, 1)
        ftem = np.zeros(12)
        Xu = self.Xu
        Du = self.Du
        Yv = self.Yv
        Dv = self.Dv
        Zw = self.Zw
        Dw = self.Dw
        Kp = self.Kp
        Dp = self.Dp
        Mq = self.Mq
        Dq = self.Dq
        Nr = self.Nr
        Dr = self.Dr

        m11 = self.m11
        m22 = self.m22
        m33 = self.m33
        m44 = self.m44
        m55 = self.m55
        m66 = self.m66

        cphi = np.cos(phi)
        sphi = np.sin(phi)
        ctheta = np.cos(theta)
        stheta = np.sin(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)

        C = np.zeros((12, 12))
        C[0, 6] = cpsi * ctheta
        C[0, 7] = -spsi * cphi + cpsi * stheta * sphi
        C[0, 8] = spsi * sphi + cpsi * stheta * cphi
        C[1, 6] = spsi * ctheta
        C[1, 7] = cpsi * cphi + spsi * stheta * sphi
        C[1, 8] = -cpsi * sphi + spsi * stheta * cphi
        C[2, 6] = -stheta
        C[2, 7] = ctheta * sphi
        C[2, 8] = ctheta * cphi
        C[3, 9] = 1
        C[3, 10] = sphi * stheta / ctheta
        C[3, 11] = cphi * stheta / ctheta
        C[4, 10] = cphi
        C[4, 11] = -sphi
        C[5, 10] = sphi / ctheta
        C[5, 11] = cphi / ctheta

        C[6, 6] = (-Xu - Du * abs(u)) / m11
        C[6, 10] = -m33 * w / m11
        C[6, 11] = m22 * v / m11
        C[7, 7] = (-Yv - Dv * abs(v)) / m22
        C[7, 9] = m33 * w / m22
        C[7, 11] = -m11 * u / m22
        C[8, 8] = (-Zw - Dw * abs(w)) / m33
        C[8, 9] = - m22 * v / m33
        C[8, 10] = m11 * u / m33

        C[9, 7] = -m33 * w / m44
        C[9, 8] = m22 * v / m44
        C[9, 9] = (-Kp - Dp * abs(p)) / m44
        C[9, 10] = -m66 * r / m44
        C[9, 11] = m55 * q / m44

        C[10, 6] = m33 * w / m55
        C[10, 8] = -m11 * u / m55
        C[10, 9] = m66 * r / m55
        C[10, 10] = (-Mq - Dq * abs(q)) / m55
        C[10, 11] = -m44 * p / m55

        C[11, 6] = -m22 * v / m66
        C[11, 7] = m11 * u / m66
        C[11, 9] = -m55 * q / m66
        C[11, 10] = m44 * p / m66
        C[11, 11] = (-Nr - Dr * abs(r)) / m66

        D = np.zeros((12, 3))
        D[6, 0] = 1 / m11
        D[8, 1] = 1 / m33
        D[11, 2] = 1 / m66

        f = np.dot(C, f) + np.dot(D, input)
        ftem[0] = f[0]
        ftem[1] = f[1]
        ftem[2] = f[2]
        ftem[3] = f[3]
        ftem[4] = f[4]
        ftem[5] = f[5]
        ftem[6] = f[6]
        ftem[7] = f[7]
        ftem[8] = f[8]
        ftem[9] = f[9]
        ftem[10] = f[10]
        ftem[11] = f[11]
        return ftem

    def step(self, u):
        sn = odeint(self.dynamics, self.s0, [0, self.dt], args=(u, ))
        self.s0 = sn[-1, :]
        r = 0
        done = False
        return self.s0, r, done, {}
    
# This class is employed to collect motion data of different robots
class data_collecter():
    def __init__(self, env_name) -> None:
        self.env_name = env_name
        np.random.seed(2022)
        random.seed(2022)
        if self.env_name.startswith("DP"):
            self.env = DP()
            self.Nstates = self.env.Nstates
            self.umax = self.env.umax # delta input
            self.umin = self.env.umin
            self.udim = 1
            self.observation_space = self.env.observation_space
            self.env.reset()
        elif self.env_name.endswith("AUV"):
            self.env = AUV()
            self.Nstates = self.env.Nstates
            self.umax = self.env.umax # delta input
            self.umin = self.env.umin
            self.udim = 2
            self.observation_space = self.env.observation_space
            self.env.reset()
        else:
            self.env = AUV_3D()
            self.Nstates = self.env.Nstates
            self.umax = self.env.umax # delta input
            self.umin = self.env.umin
            self.udim = 3
            self.observation_space = self.env.observation_space
            self.env.reset()

    def collect_koopman_data(self, traj_num):
        train_data = np.empty((traj_num, self.Nstates + self.udim + 1))
        if self.env_name.startswith("DP"):
            self.dt = 0.02
            s0 = self.env.reset()
            u10 = np.random.uniform(self.umin, self.umax)
            train_data[0, :] = np.concatenate([u10.reshape(-1), s0.reshape(-1), [0]], axis = 0).reshape(-1)
            for traj_i in range(1, traj_num):
                s0, r, done, _ = self.env.step(u10)
                u11 = np.random.uniform(self.umin, self.umax)
                u10 = u11 + u10
                u10[0] = np.min([np.max([-8, u10[0]]), 8])
                train_data[traj_i, :] = np.concatenate([u10.reshape(-1), s0.reshape(-1), [self.dt * traj_i]], axis = 0).reshape(-1)
        elif self.env_name.endswith("AUV"):
            self.dt = 0.05
            s0 = self.env.reset()
            u10 = np.random.uniform(self.umin, self.umax)
            train_data[0, :] = np.concatenate([u10.reshape(-1), s0.reshape(-1), [0]], axis = 0).reshape(-1)
            for traj_i in range(1, traj_num):
                s0, r, done, _ = self.env.step(u10)
                u11 = np.random.uniform(self.umin, self.umax)
                u10 = u11 + u10
                u10[0] = np.min([np.max([0, u10[0]]), 6])
                u10[1] = np.min([np.max([-3, u10[1]]), 3])
                train_data[traj_i, :] = np.concatenate([u10.reshape(-1), s0.reshape(-1), [self.dt * traj_i]], axis = 0).reshape(-1)
        else:
            self.dt = 0.05
            s0 = self.env.reset()
            u10 = np.random.uniform(self.umin, self.umax)
            train_data[0, :] = np.concatenate([u10.reshape(-1), s0.reshape(-1), [0]], axis = 0).reshape(-1)
            for traj_i in range(1, traj_num):
                s0, r, done, _ = self.env.step(u10)
                u11 = np.random.uniform(self.umin, self.umax)
                u10 = u11 + u10
                u10[0] = np.min([np.max([0, u10[0]]), 6])
                u10[1] = np.min([np.max([-0.2, u10[1]]), 0.2])
                u10[2] = np.min([np.max([-3, u10[2]]), 3])
                train_data[traj_i, :] = np.concatenate([u10.reshape(-1), s0.reshape(-1), [self.dt * traj_i]], axis = 0).reshape(-1)
        return train_data
