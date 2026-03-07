// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindex;

/**
 * Jostling – agitates the spindexer to prevent game-piece jams.
 *
 * <p>{@link Spindex#agitate()} is time-varying and must be called repeatedly.
 * Cancel this command when jostling is no longer needed.
 */
public class Jostling extends Command {

  private final Spindex m_spindex;

  public Jostling(Spindex spindex) {
    m_spindex = spindex;
    addRequirements(spindex);
  }

  @Override
  public void execute() {
    m_spindex.agitate();
  }

  @Override
  public void end(boolean interrupted) {
    m_spindex.rest();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
