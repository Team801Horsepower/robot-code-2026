// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Spindex;

/**
 * Jostling – runs the hopper jostle and spindexer agitation in parallel to prevent game-piece
 * jams.
 *
 * <p>Both {@link Hopper#jostle()} and {@link Spindex#agitate()} are time-varying and must be
 * called repeatedly. Cancel this command when jostling is no longer needed.
 */
public class Jostling extends Command {

  private final Hopper  m_hopper;
  private final Spindex m_spindex;

  public Jostling(Hopper hopper, Spindex spindex) {
    m_hopper  = hopper;
    m_spindex = spindex;
    addRequirements(hopper, spindex);
  }

  @Override
  public void execute() {
    m_hopper.jostle();
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
