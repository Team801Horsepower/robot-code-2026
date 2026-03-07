// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Launch;

/**
 * Shoot – runs the full launch sequence (spindexer, feeder, turret wheels) and jostles the hopper
 * until cancelled.
 *
 * <p>Pair with {@link Aim} to set hood angle and turret rotation before or during shooting.
 */
public class Shoot extends Command {

  private final Launch m_launch;
  private final Hopper m_hopper;

  public Shoot(Launch launch, Hopper hopper) {
    m_launch = launch;
    m_hopper = hopper;
    addRequirements(launch, hopper);
  }

  @Override
  public void execute() {
    m_launch.launch();
    m_hopper.jostle();
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
