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




def spin():
    dt = 1./240.
    env = environment()
    sbr = robot()
    gyro_sensor = MPUSimulated(dt)


    num_joints = p.getNumJoints(sbr.boxId)
    print("Number of joints in the robot:", num_joints)

    j0 = p.getJointInfo(sbr.boxId, 0)
    j1 = p.getJointInfo(sbr.boxId, 1)
    print("Joint 0:", j0)
    print("Joint 1:", j1)

    maxForce = 0 
    mode = p.VELOCITY_CONTROL
    # Torque control
    # p.setJointMotorControl2(sbr.boxId, 0, controlMode=mode, force=maxForce) 
    # p.setJointMotorControl2(sbr.boxId, 1, controlMode=mode, force=maxForce)

    p.setJointMotorControlArray(sbr.boxId, [0, 1], controlMode=mode, forces=[maxForce, maxForce])

    # j0s = p.getJointState(sbr.boxId, 0)
    # j1s = p.getJointState(sbr.boxId, 1)
    # print("Joint 0 state:", j0s)
    # print("Joint 1 state:", j1s)

    for i in range (10000):
        p.stepSimulation()
        print("gyro:", gyro_sensor.get_true_acc(sbr.boxId, dt, i))
        time.sleep(dt)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(sbr.boxId)
    print(cubePos,cubeOrn)
    p.disconnect()

