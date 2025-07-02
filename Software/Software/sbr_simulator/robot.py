import pybullet as p
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class robot:

    def __init__(self):
        startPos = [0,0,0.1]
        startOrientation = p.getQuaternionFromEuler([0,0,0.45])
        self.boxId = p.loadURDF("urdf/vehicle_urdf/urdf/vehicle_urdf.urdf",startPos, startOrientation)


class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output
    

class MPUSimulated:

    def __init__(self, dt):
        offset_mm = np.array([4.96, -43.24, 47.57])
        self.offset_m = offset_mm / 1000.0
        self.last_lin_vel = np.zeros(3)
        self.last_ang_vel = np.zeros(3)
        self.dt = dt

    def get_true_acc(self, robot_id, dt, i) -> tuple:
        lin_vel, ang_vel = p.getBaseVelocity(robot_id)
        pos, orn = p.getBasePositionAndOrientation(robot_id)

        r = R.from_quat(orn)
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

        return self.add_noise(accel_body, gyro_body)

    def add_noise(self, accel_body, gyro_body, noise_std=0.01):
        noise_accel = np.random.normal(0, noise_std, size=accel_body.shape)
        noise_gyro = np.random.normal(0, noise_std, size=gyro_body.shape)
        return accel_body + noise_accel, gyro_body + noise_gyro


