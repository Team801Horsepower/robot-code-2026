package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ShortCommands {
    public static final Command GatherCommand = new RunCommand(() -> RobotContainer.m_GatherSubsystem.RunGatherIntake(RobotContainer.driverController.getLeftTriggerAxis()), RobotContainer.m_GatherSubsystem);

    public static final Command ShootCommand = new SequentialCommandGroup(
        new RunCommand(() -> RobotContainer.m_FeederSubsystem.SpinFeederForward(), RobotContainer.m_FeederSubsystem),
        new RunCommand(() -> RobotContainer.m_SpindexerSubsystem.RunSpindexerForward(), RobotContainer.m_SpindexerSubsystem)
    );
}