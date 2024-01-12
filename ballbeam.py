import pybullet as p 
import pybullet_data
from time import sleep
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#targid = p.loadURDF("sphere.urdf")

angle = p.addUserDebugParameter('Motor', -0.5, 0.5, 0)
ballbeam = p.loadURDF('urdf/simple_beam.urdf', [0, 0, 0.1], useFixedBase=True)
#objects = p.loadURDF("soccerball.urdf")
number_of_joints = p.getNumJoints(ballbeam)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(ballbeam, joint_number)
    print(info[0], ": ", info[1])


hinge_index = [1]
sleep(1)

# keep this for bar to go down 
p.setJointMotorControl2(ballbeam, 1, p.VELOCITY_CONTROL, force=0)
angles = [0.75, 0.5, 0.25, 0, -0.25, -0.5, 0.75 ]
count = 0

while True:
    user_angle = p.readUserDebugParameter(angle)
    for joint_index in hinge_index:
        p.setJointMotorControl2(ballbeam, joint_index,
                                p.TORQUE_CONTROL,
                                force=user_angle)
    p.stepSimulation()

"""
while True:
    #user_angle = p.readUserDebugParameter(angle)
    #for joint_index in hinge_index:
    print(count)
    p.setJointMotorControl2(ballbeam, 2,
                                p.TORQUE_CONTROL,
                                force=angles[count])
    p.stepSimulation()
    count += 1
    if(count >= len(angles)-1):
        break
"""
