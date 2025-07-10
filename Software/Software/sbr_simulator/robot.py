import pybullet as p
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rotsc
import plotly.graph_objects as go
import pandas as pd
import plotly.io as pio

class robot:

    def __init__(self):
        startPos = [0,0,0.1]
        startOrientation = p.getQuaternionFromEuler([0,0,0.45])
        self.boxId = p.loadURDF("urdf/vehicle_urdf/urdf/vehicle_urdf.urdf",startPos, startOrientation)
        max_force = 0
        mode = p.TORQUE_CONTROL
        p.setJointMotorControlArray(self.boxId, [0, 1], controlMode=mode, forces=[max_force, max_force])


class PID:
    def __init__(self, Kp, Ki, Kd, dt, windup=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.prev_error = 0
        self.integral = 0
        self.windupmax = windup

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.potential = self.Kp * error
        self.integral += error * self.dt
        self.derivative = (error - self.prev_error) / self.dt
        
        self.antiwindup()  
        
        output = (self.potential) + (self.Ki * self.integral) + (self.Kd * self.derivative)
        self.prev_error = error
        return output

    def antiwindup(self):
        if self.windupmax != 0.0:
            if self.integral > self.windupmax:
                self.Ki = self.windupmax
            elif self.Ki < -self.windupmax:
                self.Ki = -self.windupmax


class MPUSimulated:

    def __init__(self, dt):
        offset_mm = np.array([4.96, -43.24, 47.57])
        self.offset_m = offset_mm / 1000.0
        self.last_lin_vel = np.zeros(3)
        self.last_ang_vel = np.zeros(3)
        self.dt = dt

        self.time = [dt*i for i in range(100)]

        self.accel_data = []
        self.gyro_data = []
        self.pitch_data = []


    def get_true_acc(self, robot_id, dt, i) -> tuple:
        lin_vel, ang_vel = p.getBaseVelocity(robot_id)
        pos, orn = p.getBasePositionAndOrientation(robot_id)

        r = Rotsc.from_quat(orn)
        rot_mat = r.as_matrix()  # world_to_body
        # print("Rotation matrix:", rot_mat)

        # Angular acceleration (approximate with finite difference)
        if i > 0:
            ang_acc = (np.array(ang_vel) - np.array(self.last_ang_vel)) / dt
        else:
            ang_acc = np.zeros(3)

        # Linear acceleration of the center of mass
        if i > 0:
            lin_acc_cm = (np.array(lin_vel) - np.array(self.last_lin_vel)) / dt
        else:
            lin_acc_cm = np.zeros(3)

        # Convert local offset to world frame
        r_world = r.apply(self.offset_m)

        # Rotational components at offset
        alpha_cross_r = np.cross(ang_acc, r_world)
        omega_cross_r = np.cross(ang_vel, r_world)
        omega_cross_omega_cross_r = np.cross(ang_vel, omega_cross_r)

        lin_acc_at_mpu = lin_acc_cm + alpha_cross_r + omega_cross_omega_cross_r

        accel_body = r.inv().apply(lin_acc_at_mpu + np.array([0, 0, 9.81]))  # include gravity
        gyro_body = r.inv().apply(ang_vel)  # angular velocity same at all points

        self.accel_data.append(accel_body)
        self.gyro_data.append(gyro_body)
        n_accel_body, n_gyro_body = self.add_noise(accel_body, gyro_body)

        return n_accel_body, n_gyro_body

    def add_noise(self, accel_body, gyro_body, noise_std=0.01):
        noise_accel = np.random.normal(0, noise_std, size=accel_body.shape)
        noise_gyro = np.random.normal(0, noise_std, size=gyro_body.shape)
        return accel_body + noise_accel, gyro_body + noise_gyro

    def get_pitch(self, accel_body):
        # Calculate pitch from accelerometer data
        pitch = np.arctan2(accel_body[0], np.sqrt(accel_body[1]**2 + accel_body[2]**2))
        self.pitch_data.append(pitch)
        return pitch

    def plot_pid(self):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.time, y=self.pitch_data, mode='lines', name='Pitch'))
        fig.update_layout(
            title='Pitch vs Time',
            xaxis_title='Time (s)',
            yaxis_title='Pitch (rad)',
            template='plotly_white',
            showlegend=True
        )
        fig.show()
        fig.write_image("pitch_vs_time.png")

    def save_data(self, time_series, pitch_lst):

        df = pd.DataFrame({'Time (s)': time_series})
        print(time_series)
        print(pitch_lst)
        for data in pitch_lst:
            print(data)
            for key, value in data.items():
                df[f'Pitch ({key})'] = value

        df.to_excel("pid_data.xlsx", index=False)

class KalmanFilter:
    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F = F  # State transition model
        self.B = B  
        self.H = H  
        self.Q = Q  
        self.R = R  
        self.x = x0  
        self.P = P0  
    def predict(self, u):
        # Predict the state and state covariance
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x
    def update(self, z):
        # Compute the Kalman gain
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))      
        # Update the state estimate and covariance matrix
        y = z - np.dot(self.H, self.x)  
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
        return self.x
