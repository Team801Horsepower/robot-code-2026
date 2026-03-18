// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gather;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Spindex;

/**
 * Shoot – runs the full launch sequence (spindexer, feeder, gather) until cancelled.
 *
 * <p>Pair with {@link Aim} to set hood angle and turret rotation before or during shooting.
 */
public class Shoot extends Command {

  private final Launch m_launch;

  public Shoot(Launch launch, Gather gather, Spindex spindex, Feeder feeder) {
    m_launch = launch;
    addRequirements(launch, gather, spindex, feeder);
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
