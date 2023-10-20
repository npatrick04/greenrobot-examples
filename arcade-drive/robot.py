#!/usr/bin/env python3
"""
    This is a demo program showing use of a differential drive in a green routine.
"""

import wpilib
import wpilib.drive

import greenrobot
from greenrobot import GreenRobot, tick

class MyRobot(GreenRobot):

    def robotInit(self):
        GreenRobot.robotInit(self)

        # create motor controller objects
        m_left = wpilib.Talon(0)
        m_right = wpilib.Talon(1)
        # object that handles basic drive operations
        self.myRobot = wpilib.drive.DifferentialDrive(m_left, m_right)
        self.myRobot.setExpiration(0.1)

        # joystick #0
        self.stick = wpilib.Joystick(0)

        # Disable motor safety so that the greenlet can operate intuitively
        m_left.setSafetyEnabled(False)
        m_right.setSafetyEnabled(False)

        # Application routines
        GreenRobot.schedule(greenrobot.State.Teleop,staticmethod(self.teleopRoutine))

    def robotPeriodic(self):
        GreenRobot.run()

    def teleopRoutine(self):
        while True:
            self.myRobot.arcadeDrive(
                self.stick.getRawAxis(0),
                self.stick.getRawAxis(1),
                True
            )
            tick(1)

if __name__ == "__main__":
    wpilib.run(MyRobot)
