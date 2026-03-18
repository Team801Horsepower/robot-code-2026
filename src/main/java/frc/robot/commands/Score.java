// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Gather;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Spindex;

/**
 * Score – runs the full score cycle via {@link Manipulator#score()}.
 *
 * <p>Primarily intended for autonomous use. Cancel or timeout this command
 * when scoring is complete.
 */
public class Score extends Command {

  private final Manipulator m_manipulator;

  public Score(Manipulator manipulator, Gather gather, Hopper hopper,
      Spindex spindex, Feeder feeder) {
    m_manipulator = manipulator;
    addRequirements(manipulator, gather, hopper, spindex, feeder);
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
