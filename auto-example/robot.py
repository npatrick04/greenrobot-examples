#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import wpilib.drive
from wpilib import SmartDashboard
from enum import Enum

import greenrobot
from greenrobot import GreenRobot, tick

class armCommand(Enum):
    down = 0
    up = 1

class MyRobot(GreenRobot):

    def robotInit(self):
        GreenRobot.robotInit(self)

        self.commandArm = armCommand.down
        self.isArmUp = False
        self.isArmDown = True
        self.armPosition = 0
        self.ARM_UP = 90
        self.ARM_DOWN = 0

        self.isClawOpen = False
        self.isClawClosed = False
        self.clawPosition = 0

        # Sequence stuff
        self.sequenceInProg = False
        self.runningCancellable = False
        self.runningCancellableID = 0
        
        self.left_motor = wpilib.Spark(0)
        self.right_motor = wpilib.Spark(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.stick = wpilib.Joystick(1)

        self.arm_motor = wpilib.Spark(2)
        self.claw_motor = wpilib.Spark(3)

        # Disable motor safety so that the greenlet can operate intuitively
        self.left_motor.setSafetyEnabled(False)
        self.right_motor.setSafetyEnabled(False)
        self.arm_motor.setSafetyEnabled(False)
        self.claw_motor.setSafetyEnabled(False)
        self.drive.setSafetyEnabled(False)

        # Application routines
        GreenRobot.schedule(greenrobot.State.Autonomous,staticmethod(self.autoRoutine))
        GreenRobot.schedule(greenrobot.State.Autonomous,staticmethod(self.armRoutine))

        GreenRobot.schedule(greenrobot.State.Teleop,staticmethod(self.teleopRoutine))
        GreenRobot.schedule(greenrobot.State.Teleop,staticmethod(self.armRoutine))

        # Sim stuff
        GreenRobot.schedule(greenrobot.State.Autonomous,staticmethod(self.simArm))
        GreenRobot.schedule(greenrobot.State.Autonomous,staticmethod(self.simClaw))

        GreenRobot.schedule(greenrobot.State.Teleop,staticmethod(self.simArm))
        GreenRobot.schedule(greenrobot.State.Teleop,staticmethod(self.simClaw))


    def robotPeriodic(self):
        GreenRobot.run()

        SmartDashboard.putBoolean("Arm Up", self.isArmUp)
        SmartDashboard.putBoolean("Arm Down", self.isArmDown)
        SmartDashboard.putNumber("Arm Position", self.armPosition)
        SmartDashboard.putBoolean("Claw Open", self.isClawOpen)
        SmartDashboard.putBoolean("Claw Closed", self.isClawClosed)
        SmartDashboard.putNumber("Claw Position", self.clawPosition)
        SmartDashboard.putBoolean("Sequence In Prog", self.sequenceInProg)

    def autoRoutine(self):
        print("Starting")
        self.clawOpen()

        # Drive to the place we want to work
        self.drive.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
        tick(50*2)

        print("Stopping")
        self.drive.arcadeDrive(0, 0)  # Stop robot

        # Move the arm up at half speed until stopped while opening the claw
        self.armUp()

        # Testing shows we can execute 3 cycles before autonomous is over
        # This cycle starts with the arm in transit to the up position, and claw open
        for i in range(3):
            self.loadCycle(i)

        print("Done with autonomous")

        # Could wait forever...
        # tick(WAIT_FOREVER)

        # ... but instead, we'll exit here, and just restart if auto mode is restarted

    def loadCycle(self, i):
        self.sequenceInProg = True
        print("{}: Begin cycle {}".format(GreenRobot.getTickCount(),i))
        print("{}: Wait for arm to be up".format(GreenRobot.getTickCount()))
        self.waitForArm(armCommand.up)

        # Now in position to close the claw on the thing
        print("{}: Pick the thing up".format(GreenRobot.getTickCount()))
        self.clawClose()
        self.waitForClawClosed()

        # Rotate and put arm down
        print("{}: Rotate clockwise".format(GreenRobot.getTickCount()))
        self.drive.arcadeDrive(0,0.5)
        self.armDown()

        # Rotation takes 3 seconds
        tick(50*3)
        self.drive.arcadeDrive(0,0)

        self.waitForArm(armCommand.down)

        # Deposit the thing
        print("{}: Deposit the thing".format(GreenRobot.getTickCount()))
        self.clawOpen()

        # Rotate back and put the arm up
        print("{}: Rotate counter-clockwise".format(GreenRobot.getTickCount()))
        self.drive.arcadeDrive(0,-0.5)

        # Rotation takes 3 seconds
        tick(50*3)
        self.drive.arcadeDrive(0,0)

        # Only for automated sequence...
        if i < 10:
            self.armUp()
            
        self.sequenceInProg = False

    def teleopRoutine(self):
        while True:
            self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())

            armButton = self.stick.getRawButton(1)
            clawButton = self.stick.getRawButton(2)
            loadCommand = self.stick.getRawButtonPressed(3)
            loadCommandCancellable = self.stick.getRawButtonPressed(4)

            if loadCommand:
                # Run the automated sequence once
                print("Start the auto routine - not cancellable")
                self.armUp()
                self.loadCycle(10)
                print("Non-cancellable routine completed")

            elif loadCommandCancellable:
                # Command for cancellable has been triggered

                if self.runningCancellable:
                    # Cancellable routine is already running, kill it
                    print("Cancel the auto routine")
                    GreenRobot.remove(self.runningCancellableID)
                    self.runningCancellable = False

                else:
                    # Run the automated sequence once
                    print("Start the auto routine - cancellable")
                    self.armUp()
                    self.runningCancellableID = GreenRobot.schedule(greenrobot.State.Current,staticmethod(self.loadCycle),11)
                    self.runningCancellable = True
                
            elif self.runningCancellable == True:
                # Check if the routine finished
                if not GreenRobot.is_alive(self.runningCancellableID):
                    print("Cancellable routine completed")
                    self.runningCancellable = False
                
            else:
                if armButton == True:
                    self.armUp()
                else:
                    self.armDown()
            
                if clawButton == True:
                    self.clawClose()
                else:
                    self.clawOpen()
            
            tick(1)

    ########## ARM LOGIC #############

    def waitForArm(self, position):
        if position == armCommand.up:
            while self.isArmUp == False:
                tick(1)
        if position == armCommand.down:
            while self.isArmDown == False:
                tick(1)

    def armUp(self):
        self.commandArm = armCommand.up

    def armDown(self):
        self.commandArm = armCommand.down

    def armRoutine(self):
        while True:
            # Do the commands
            if self.commandArm == armCommand.up:
                if self.armPosition != self.ARM_UP:
                    if self.arm_motor.get() != 1:
                        print("Arm is {}, commanded up".format(self.arm_motor.get()))
                        self.arm_motor.set(1)
            if self.commandArm == armCommand.down:
                if self.armPosition != self.ARM_DOWN:
                    if self.arm_motor.get() != -1:
                        print("Arm is {}, commanded down".format(self.arm_motor.get()))
                        self.arm_motor.set(-1)

            # Stop the commands
            if self.armPosition == self.ARM_UP:
                if self.commandArm == armCommand.up:
                    if self.isArmUp == False:
                        print("Arm is now up")
                    self.isArmUp = True
                    self.arm_motor.set(0)

            if self.armPosition == self.ARM_DOWN:
                if self.commandArm == armCommand.down:
                    if self.isArmDown == False:
                        print("Arm is now down")
                    self.isArmDown = True
                    self.arm_motor.set(0)

            if self.armPosition > self.ARM_DOWN:
                self.isArmDown = False
            if self.armPosition < self.ARM_UP:
                self.isArmUp = False

            tick(1)


   ########## CLAW LOGIC #############

    def clawOpen(self):
        if self.isClawOpen == False:
            # print("Open Claw")
            self.claw_motor.set(0.5)

    def clawClose(self):
        if self.isClawClosed == False:
            # print("Close Claw")
            self.claw_motor.set(-0.5)

    def waitForClawOpen(self):
        while self.isClawOpen == False:
            tick(1)

    def waitForClawClosed(self):
        while self.isClawClosed == False:
            tick(1)

   ########## SIM LOGIC #############
            

    def simArm(self):
        while True:
            # Simulate arm motion
            armCommand = self.arm_motor.get()

            if armCommand > 0.0:
                self.armPosition = self.armPosition + 0.45
            if armCommand < 0.0:
                self.armPosition = self.armPosition - 0.45

            if self.armPosition > self.ARM_UP:
                self.armPosition = self.ARM_UP

            if self.armPosition < self.ARM_DOWN:
                self.armPosition = self.ARM_DOWN
            tick(1)

    def simClaw(self):
        self.clawPosition = 0
        while True:
            if self.claw_motor.get() > 0:
                self.clawPosition += 1
            if self.claw_motor.get() < 0:
                self.clawPosition -= 1

            if self.clawPosition >= 25:
                self.clawPosition = 25
                self.isClawOpen = True
            elif self.clawPosition <= 0:
                self.clawPosition = 0
                self.isClawClosed = True
            else:
                self.isClawOpen = False
                self.isClawClosed = False
            tick(1)


if __name__ == "__main__":
    wpilib.run(MyRobot)
