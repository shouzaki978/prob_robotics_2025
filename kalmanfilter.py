import numpy as np
import matplotlib.pyplot as plt
import numpy.random as random
import time

dt = 0.1 #刻み時間
max_time = 50 #実行時間
sigma_a = 1.0 #移動量のノイズ
sigma_z = 2.0 #センサのノイズ
u = 0.5 #制御入力


#正規分布に従った乱数
def norm(mean=0.0, std=1.0, size=1):
    return random.normal(mean, std, size) #第二引数は標準偏差

def init_params(dt, sigma_a, sigma_z):
    
    A = np.matrix([[1, dt],
                   [0, 1]]) #状態遷移行列
    
    B = np.matrix([[dt**2 / 2],
                   [dt]])  #制御入力

    G = np.matrix([[dt**2 / 2],
                   [dt]]) #ノイズ

    H = np.matrix([1, 0]) #観測行列

    Q = (sigma_a ** 2) * G *G.T #ノイズ共分散行列

    R = sigma_z ** 2 #観測ノイズ共分散行列

    I = np.eye(2) #単位行列

    x0 = np.matrix([[0.0],
                    [0.0]]) #初期状態

    p0 = np.zeros((2, 2)) #初期誤差共分散行列
    
    return A, B, G, H, Q, R, I, x0, p0


def kalman_step(x_est, P_est, x_true, A, B, G, H, Q, R, I, u, sigma_a, sigma_z):
    # 観測
    z = H @ x_true + norm(0, sigma_z)

    # 予測
    x_pred = A @ x_est + B * u
    P_pred = A @ P_est @ A.T + Q

    # 真値更新
    w = norm(0, sigma_a)
    x_true = A @ x_true + B * u + G * w

    # 更新
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_est = x_pred + K @ (z - H @ x_pred)
    P_est = (I - K @ H) @ P_pred

    return x_est, P_est, x_true, z


def run_simulation(dt, steps, u, sigma_a, sigma_z):
    A, B, G, H, Q, R, I, x_est, P_est = init_params(dt, sigma_a, sigma_z)
    x_true = x_est.copy()

    gt, obs, est, t = [], [], [], []

    for k in range(steps):
        x_est, P_est, x_true, z = kalman_step(
            x_est, P_est, x_true,
            A, B, G, H, Q, R, I,
            u, sigma_a, sigma_z
        )

        gt.append(float(x_true[0]))
        obs.append(float(z))
        est.append(float(x_est[0]))
        t.append(k * dt)

    return np.array(t), np.array(gt), np.array(obs), np.array(est)


# ================= 実行 =================
time, gt, obs, est = run_simulation(dt, max_time, u, sigma_a, sigma_z)

print("観測誤差:", np.sum((gt - obs) ** 2))
print("カルマン推定誤差:", np.sum((gt - est) ** 2))

plt.plot(time, gt, color="blue", marker="", label="ground truth")
plt.plot(time, est, color="red", marker="+", label="estimation")
plt.plot(time, obs, color="green", marker="", label="observation")
plt.legend()
plt.show()

    
    