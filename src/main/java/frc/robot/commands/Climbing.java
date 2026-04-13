// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Climbing – engages the climb mechanism and enables vertical displacement
 * compensation (pitch/roll/elevation) on the turret.
 *
 * <p>While this command is running, the climb motor drives to the fully-climbed
 * position and the turret accounts for robot tilt and height changes. When the
 * command ends, the climb motor returns to rest and the turret uses flat-ground
 * aiming only.
 */
public class Climbing extends Command {

  private final TurretSubsystem m_turret;
  private final Climb m_climb;

  public Climbing(TurretSubsystem turret, Climb climb) {
    m_turret = turret;
    m_climb = climb;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    m_turret.setClimbing(true);
    m_climb.climb();
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.setClimbing(false);
    m_climb.rest();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
