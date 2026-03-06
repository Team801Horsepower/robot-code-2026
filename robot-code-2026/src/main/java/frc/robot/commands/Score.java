// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

/**
 * Score – runs the full score cycle via {@link Manipulator#score()}.
 *
 * <p>Primarily intended for autonomous use. Cancel or timeout this command
 * when scoring is complete.
 */
public class Score extends Command {

  private final Manipulator m_manipulator;

  public Score(Manipulator manipulator) {
    m_manipulator = manipulator;
    addRequirements(manipulator);
  }

  @Override
  public void execute() {
    m_manipulator.score();
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
