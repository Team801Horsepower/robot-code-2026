// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Climb – enables vertical displacement compensation (pitch/roll/elevation) on the turret.
 *
 * <p>While this command is running, the turret accounts for robot tilt and height changes.
 * When not running, the turret uses flat-ground aiming only.
 */
public class Climb extends Command {

  private final TurretSubsystem m_turret;

  public Climb(TurretSubsystem turret) {
    m_turret = turret;
  }

  @Override
  public void initialize() {
    m_turret.setClimbing(true);
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.setClimbing(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
