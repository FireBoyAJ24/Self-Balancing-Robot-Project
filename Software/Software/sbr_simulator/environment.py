import pybullet as p
import time
import pybullet_data
from robot import *


class environment():

    def __init__(self):
        self.physicsClient = p.connect(p.GUI, options="--opengl2")
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)
        self.planeId = p.loadURDF("plane.urdf")

        p.setPhysicsEngineParameter(enableFileCaching=0) # Disable URDF caching



def run_sim(dt, pid_const, gyro_sensor):
    
    env = environment()
    sbr = robot()
    
    p.setRealTimeSimulation(0)  # Disable real-time simulation
    p.setTimeStep(1/240.)  # Set time step for the simulation



    setpoint = 0.0  # Desired pitch angle

    mode = p.TORQUE_CONTROL

    for i in range (10000):
        p.stepSimulation()

        # Get MPU data
        accel_body, gyro_body = gyro_sensor.get_true_acc(sbr.boxId, dt, i)

        # Get pitch from accelerometer data
        # Note: The pitch may be incorrect if the robot is not upright
        pitch = gyro_sensor.get_pitch(accel_body)

        # PID control
        pid = PID(Kp=pid_const[0], Ki=pid_const[1], Kd=pid_const[2], dt=dt, windup=0.0)
        torque_output = pid.compute(setpoint, pitch)
        print(torque_output)

        # Apply torque to the robot's joints
        p.setJointMotorControlArray(sbr.boxId, [0, 1], controlMode=mode, forces=[torque_output, torque_output])

        # Sleep to maintain simulation time
        time.sleep(dt)
    
    p.disconnect()


def pid_brute_force():
    dt = 1/240.

    pid_const = np.array([0.0, 0.0, 0.0])  # Kp, Ki, Kd
    pid_sets = list()

    gyro_sensor = MPUSimulated(dt)


    while (pid_const[0] <= 1.0):
        pid_const[1] = 0.0

        while (pid_const[1] <= 1.0):
            pid_const[2] = 0.0

            while (pid_const[2] <= 1.0):
                run_sim(dt, pid_const, gyro_sensor)


                # Label the PID constants
                label = f"Kp: {pid_const[0]:.01}, Ki: {pid_const[1]:.01}, Kd: {pid_const[2]:.01}"
                
                data = {label: gyro_sensor.pitch_data.copy()}

                pid_sets.append(data)

                gyro_sensor.pitch_data.clear()

                # Increment Kd
                pid_const[2] += 0.1

            # Increment Ki
            pid_const[1] += 0.1

        # Increment Kp
        pid_const[0] += 0.1

    time = [dt * i for i in range(100)]

    gyro_sensor.save_data(time, pid_sets)


def spin():
    dt = 1/240.  # Simulation time step

    # Uncomment the line below to run the brute force PID tuning
    # pid_brute_force()

    # Example usage of the run_sim function with a specific PID constant
    pid_const = np.array([1.0, 300, 0.31])  # Example PID constants
    gyro_sensor = MPUSimulated(dt)
    
    run_sim(dt, pid_const, gyro_sensor)

    print("Plotting PID")
    gyro_sensor.plot_pid()




# env = environment()
# sbr = robot()
# gyro_sensor = MPUSimulated(dt)

# setpoint = 0.0  # Desired pitch angle


# mode = p.VELOCITY_CONTROL

# for i in range (100):
#     p.stepSimulation()

#     # Get MPU data
#     accel_body, gyro_body = gyro_sensor.get_true_acc(sbr.boxId, dt, i)
#     print("gyro:", (accel_body, gyro_body))

#     # Get pitch from accelerometer data
#     # Note: The pitch may be incorrect if the robot is not upright
#     pitch = gyro_sensor.get_pitch(accel_body)
#     print("Pitch:", pitch)

#     # PID control
#     pid = PID(Kp=1, Ki=0, Kd=0, dt=dt)
#     torque_output = pid.compute(setpoint, pitch)

#     # Apply torque to the robot's joints
#     p.setJointMotorControlArray(sbr.boxId, [0, 1], controlMode=mode, forces=[torque_output, torque_output])

#     # Sleep to maintain simulation time
#     time.sleep(dt)

# p.disconnect()

# gyro_sensor.plot_pid()

# num_joints = p.getNumJoints(sbr.boxId)
# print("Number of joints in the robot:", num_joints)

# j0 = p.getJointInfo(sbr.boxId, 0)
# j1 = p.getJointInfo(sbr.boxId, 1)
# print("Joint 0:", j0)
# print("Joint 1:", j1)

# Torque control
# p.setJointMotorControl2(sbr.boxId, 0, controlMode=mode, force=maxForce) 
# p.setJointMotorControl2(sbr.boxId, 1, controlMode=mode, force=maxForce)

# j0s = p.getJointState(sbr.boxId, 0)
# j1s = p.getJointState(sbr.boxId, 1)
# print("Joint 0 state:", j0s)
# print("Joint 1 state:", j1s)