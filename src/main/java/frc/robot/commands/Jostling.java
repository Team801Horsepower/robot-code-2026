// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Possession;

/**
 * Jostling – oscillates the hopper between its two jostle endpoints to unstick game pieces.
 *
 * <p>All state-machine logic lives in {@link Possession}; this command is a thin wrapper that
 * starts, advances, and ends the jostle routine.
 */
public class Jostling extends Command {

  private final Possession m_possession;

  public Jostling(Possession possession, Hopper hopper) {
    m_possession = possession;
    addRequirements(possession, hopper);
  }

  @Override
  public void initialize() {
    m_possession.startJostle();
  }

  @Override
  public void execute() {
    m_possession.advanceJostle();
  }

  @Override
  public void end(boolean interrupted) {
    m_possession.endJostle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
