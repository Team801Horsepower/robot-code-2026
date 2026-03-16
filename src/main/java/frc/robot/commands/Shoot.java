// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launch;

/**
 * Shoot – runs the full launch sequence (spindexer, feeder, gather) until cancelled.
 *
 * <p>Pair with {@link Aim} to set hood angle and turret rotation before or during shooting.
 */
public class Shoot extends Command {

  private final Launch m_launch;

  public Shoot(Launch launch) {
    m_launch = launch;
    addRequirements(launch);
  }

  @Override
  public void execute() {
    m_launch.launch();
  }

  @Override
  public void end(boolean interrupted) {
    m_launch.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
