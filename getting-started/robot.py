#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive

import greenrobot
from greenrobot import GreenRobot, tick

class MyRobot(GreenRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        GreenRobot.robotInit(self)

        self.leftDrive = wpilib.PWMSparkMax(0)
        self.rightDrive = wpilib.PWMSparkMax(1)
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftDrive, self.rightDrive
        )
        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

        # Disable the motor watchdog timer
        self.robotDrive.setSafetyEnabled(False)

        # Application routines
        GreenRobot.schedule(greenrobot.State.Autonomous,staticmethod(self.autonomousRoutine))
        GreenRobot.schedule(greenrobot.State.Teleop,staticmethod(self.teleopRoutine))
        GreenRobot.schedule(greenrobot.State.Test,staticmethod(self.testRoutine))

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousRoutine(self):
        """This function is a green routine that is scheduled in autonomous mode."""

        # Drive forwards half speed, make sure to turn input squaring off
        self.robotDrive.arcadeDrive(0.5, 0, squareInputs=False)

        # Wait 2 seconds
        tick(2*50)
        
        self.robotDrive.stopMotor()  # Stop robot

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopRoutine(self):
        """This function is a green routine that is scheduled in teleop mode."""
        while True:
            self.robotDrive.arcadeDrive(
                -self.controller.getLeftY(), -self.controller.getRightX()
            )
            tick(1)

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testRoutine(self):
        """This function is a green routine that is scheduled in test mode."""
        while True:
            tick(1)


if __name__ == "__main__":
    wpilib.run(MyRobot)
