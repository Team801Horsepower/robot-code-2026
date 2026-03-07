// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascension;
import frc.robot.subsystems.Hopper;

/**
 * ClimbL1 – retracts the hopper then drives the robot to the Level 1 climb position.
 *
 * <p>Sequence:
 * <ol>
 *   <li>Retract hopper (clear path for climber).
 *   <li>Extend climber to L1 setpoint via {@link Ascension#levelOne()}.
 * </ol>
 *
 * <p>The command finishes when the climber reports it is at the L1 setpoint.
 */
public class ClimbL1 extends Command {

  private final Hopper    m_hopper;
  private final Ascension m_ascension;

  public ClimbL1(Hopper hopper, Ascension ascension) {
    m_hopper    = hopper;
    m_ascension = ascension;
    addRequirements(hopper, ascension);
  }

  @Override
  public void execute() {
    m_hopper.retract();
    m_ascension.levelOne();
  }

  @Override
  public boolean isFinished() {
    // Climber is at L1 setpoint and hopper is retracted.
    return m_ascension.isAtSetpoint();
  }
}
