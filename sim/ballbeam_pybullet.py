#!/usr/bin/env python3

"""
Based off of
    - https://github.com/erwincoumans/pybullet_robots/blob/master/turtlebot.py
    - https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-2-a1441b9a4d8e
Resources used for lidar: 
    - https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py
    - https://github.com/axelbr/racecar_gym/blob/master/racecar_gym/bullet/sensors.py
Resources used for camera:
    - https://www.programcreek.com/python/example/122153/pybullet.computeViewMatrixFromYawPitchRoll

"""

import pybullet as p
import numpy as np
import time

class BallBeamBotPybullet:
    def __init__(self, gui: bool = True) -> None:
        """class to spawn in and control arbot
        """

        p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(1)
        p.setTimeStep(1. / 30)

        beam_urdf_path = "env/ballbeam/ball_beam.urdf"
        ball_urdf_path = "env/ball/ball.urdf"

        self.bot = p.loadURDF(beam_urdf_path, [0, 0, 0], useFixedBase=True)
        self.ball = p.loadURDF(ball_urdf_path, [0, 0, 0.6])

        self.gui = gui

        self._hit_color = [1, 0, 0]
        self._miss_color = [0, 1, 0]
        self._ray_ids = None

    def apply_action(self, action: int) -> None:
        """
        Performs action

        :param action: tuple consisting of translation and rotation
        """
        p.setJointMotorControl2(
        self.bot, 1, p.POSITION_CONTROL, targetPosition=action, force=1000
        )

    def get_current_angle(self) -> int:
        """
        """
        return p.getEulerFromQuaternion(p.getLinkState(self.bot, 2)[1])[1]


    def measure_distance(self) -> list:
        """simulate lidar measurement
        """

        sensor_range = 1

        ball_translation, _ = p.getBasePositionAndOrientation(
            self.ball
        )
        ray_angle = -self.get_current_angle()

        print(ray_angle)
        ray_from = ball_translation
        ray_to = ball_translation + sensor_range * np.array([np.cos(ray_angle), 0, np.sin(ray_angle)])


        result = p.rayTest(ray_from, ray_to)

        hitObjectUid = result[0][0]

        if self.gui:
            if self._ray_ids is None:
                self._ray_ids = p.addUserDebugLine(ray_from, ray_to, self._miss_color)

            if (hitObjectUid < 0):
                p.addUserDebugLine(
                    ray_from,
                    ray_to,
                    self._miss_color,
                    replaceItemUniqueId=self._ray_ids
                )
            else:
                hit_location = result[0][3]
                p.addUserDebugLine(
                    ray_from,
                    hit_location,
                    self._hit_color,
                    replaceItemUniqueId=self._ray_ids
                )

        return np.array(result, dtype=object)[:, 2]

if __name__=='__main__':
    bot = BallBeamBotPybullet(True)
    angle = p.addUserDebugParameter("Motor", -0.2, 0.2, 0)

    while(True):
        bot.apply_action(p.readUserDebugParameter(angle))
        bot.measure_distance()
        time.sleep(1. / 240)