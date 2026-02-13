#!/usr/bin/env python3

import wpilib
from subsystems import drive
from commands2 import CommandScheduler
from wpimath.geometry import Transform2d
import time


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.scheduler = CommandScheduler()
        self.drive = drive.Drive(self.scheduler)

    def robotPeriodic(self):
        pass

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        self.start_time = time.time()

    def teleopPeriodic(self):
        if time.time() - self.start_time < 5:
            drive_input = Transform2d(5, 0, 0)
        else:
            drive_input = Transform2d(0, 0, 0)

        self.drive.drive(drive_input, field_oriented=False)

    def teleopExit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)