// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.GatherSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  GatherSubsystem gatherSubsystem;
  FeederSubsystem feederSubsystem;
  SpindexerSubsystem spindexerSubsystem;
  TurretSubsystem turretSubsystem;
  Joystick driverController;

  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gatherSubsystem, feederSubsystem, spindexerSubsystem, turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gatherSubsystem.GatherForward();
    feederSubsystem.Feed();
    spindexerSubsystem.SpindexerForward();
    turretSubsystem.HoodAim();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gatherSubsystem.StopGather();
    feederSubsystem.FeederStop();
    spindexerSubsystem.SpindexerStop();
    turretSubsystem.HoodReset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
