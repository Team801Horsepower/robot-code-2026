#!/usr/bin/env python3

import wpilib
from subsystems import drive
from subsystems import nt_core_test
from commands2 import CommandScheduler
from commands2 import Command
from utils.constants import OperatorConstants


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.scheduler = CommandScheduler.getInstance()
        self.driver_controller = wpilib.XboxController(
            OperatorConstants.DRIVER_XBOX_PORT
        )
        self.drive: drive.Drive | None = None
        self._drive_faulted = False
        try:
            self.drive = drive.Drive(self.scheduler)
        except Exception as exc:
            wpilib.reportError(
                f"Drive subsystem failed to initialize. "
                f"Teleop will continue without swerve control: {exc}",
                False,
            )
        self.questnav_bridge = nt_core_test.QuestNavNtBridge()
        self.autonomous_command: Command | None = None

    def robotPeriodic(self):
        self.scheduler.run()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def autonomousInit(self):
        self.autonomous_command = None
        if self.drive is not None and not self._drive_faulted:
            try:
                self.autonomous_command = self.drive.get_autonomous_command()
            except Exception as exc:
                self._drive_faulted = True
                wpilib.reportError(
                    f"Failed to build autonomous drive command; disabling drive: {exc}",
                    False,
                )

        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        pass

    def teleopInit(self):
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None
        self.questnav_bridge.on_teleop_enable()

    def teleopPeriodic(self):
        self.questnav_bridge.update()
        # Right stick controls translation; left X controls robot rotation.
        x_displacement = -self.driver_controller.getRightY()
        y_displacement = -self.driver_controller.getRightX()
        rotation = -self.driver_controller.getLeftX()
        if self.drive is None or self._drive_faulted:
            return

        try:
            self.drive.drive(
                x_displacement=x_displacement,
                y_displacement=y_displacement,
                rotation=rotation,
                field_oriented=False,
            )
        except Exception as exc:
            self._drive_faulted = True
            wpilib.reportError(
                f"Drive call failed during teleop; drive disabled for this boot: {exc}",
                False,
            )

    def teleopExit(self):
        self.questnav_bridge.on_teleop_disable()

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
