// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Possession;

/**
 * Jostling – oscillates the hopper between {@link HopperConstants#kExtendedSetpoint}
 * and {@link HopperConstants#kJostleSetpoint} under standard extend PID control
 * to unstick game pieces.
 *
 * <p>While active, signals {@link Possession} to bypass its "hopper must be
 * extended before gathering" guard so the hopper can move freely within the
 * jostle range. On end, parks at the fully extended setpoint.
 */
public class Jostling extends Command {

  private final Possession m_possession;
  private double m_target;

  public Jostling(Possession possession, Hopper hopper) {
    m_possession = possession;
    addRequirements(possession, hopper);
  }

  @Override
  public void initialize() {
    m_possession.setJostling(true);
    m_target = HopperConstants.kJostleSetpoint;
    m_possession.jostleTo(m_target);
  }

  @Override
  public void execute() {
    if (m_possession.isHopperAt(m_target, HopperConstants.kJostleTolerance)) {
      m_target = (m_target == HopperConstants.kJostleSetpoint)
          ? HopperConstants.kExtendedSetpoint
          : HopperConstants.kJostleSetpoint;
      m_possession.jostleTo(m_target);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_possession.setJostling(false);
    m_possession.jostleTo(HopperConstants.kExtendedSetpoint);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
