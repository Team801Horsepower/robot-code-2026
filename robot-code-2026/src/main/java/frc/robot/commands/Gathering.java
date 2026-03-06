// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Possession;

/**
 * Gathering – continuously runs the intake sequence (extend hopper + spin gatherer).
 *
 * <p>This command runs indefinitely; cancel it when the driver releases the button.
 */
public class Gathering extends Command {

  private final Possession m_possession;

  public Gathering(Possession possession) {
    m_possession = possession;
    addRequirements(possession);
  }

  @Override
  public void execute() {
    m_possession.possess();
  }

  @Override
  public void end(boolean interrupted) {
    m_possession.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
