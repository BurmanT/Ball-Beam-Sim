import pybullet as p
import pybullet_data
from time import sleep

p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# targid = p.loadURDF("sphere.urdf")

angle = p.addUserDebugParameter("Motor", -0.5, 0.5, 0)
ballbeam = p.loadURDF("ball_beam.urdf", [0, 0, 0.1], useFixedBase=True)

ball = p.loadURDF("ball.urdf", [0, 0, 0.75])

number_of_joints = p.getNumJoints(ballbeam)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(ballbeam, joint_number)
    print(info[0], ": ", info[1])


hinge_index = [1]
sleep(1)

# keep this for bar to go down
p.setJointMotorControl2(ballbeam, 1, p.VELOCITY_CONTROL, force=0)
count = 0

while True:
    user_angle = p.readUserDebugParameter(angle)
    p.setJointMotorControl2(
        ballbeam, 1, p.POSITION_CONTROL, targetPosition=user_angle, force=1000
    )
    p.stepSimulation()
    sleep(1.0 / 240.0)
