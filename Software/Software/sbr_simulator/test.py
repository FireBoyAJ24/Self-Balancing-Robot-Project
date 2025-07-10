import pybullet as p
import pybullet_data
import time

client = p.connect(p.GUI)
if client < 0:
    raise RuntimeError("Failed to connect to PyBullet GUI.")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", [0, 0, 1])

for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
