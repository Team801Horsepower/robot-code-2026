package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;

public class ShortCommands {
    public static final Command GatherCommand = new RunCommand(() -> RobotContainer.m_GatherSubsystem.RunGatherIntake(RobotContainer.driverController.getLeftTriggerAxis()), RobotContainer.m_GatherSubsystem);
}