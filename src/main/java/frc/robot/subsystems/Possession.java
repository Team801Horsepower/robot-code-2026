// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GatherConstants;
import frc.robot.Constants.HopperConstants;

/**
 * Possession – composite that coordinates the Hopper and Gather subsystems to intake game pieces.
 *
 * <p>The hopper must be extended before the gatherer can run. {@link #possess()} enforces this.
 */
public class Possession extends SubsystemBase {

  private final Hopper m_hopper;
  private final Gather m_gather;
  private boolean m_jostling = false;
  private double m_jostleTarget = HopperConstants.kJostleSetpoint;

  public Possession(Hopper hopper, Gather gather) {
    m_hopper = hopper;
    m_gather = gather;
  }

  /** Begins jostling: bypasses the extend guard and commands the lower jostle setpoint. */
  public void startJostle() {
    m_jostling = true;
    m_jostleTarget = HopperConstants.kJostleSetpoint;
    m_hopper.jostleTo(m_jostleTarget);
  }

  /**
   * Advances the jostle state machine. Flips the hopper target between the two jostle endpoints
   * when the current endpoint is reached. Only re-issues {@code jostleTo} on a flip, so repeated
   * calls within tolerance do not thrash the PID setpoint.
   */
  public void advanceJostle() {
    if (m_hopper.isAt(m_jostleTarget, HopperConstants.kJostleTolerance)) {
      m_jostleTarget = (m_jostleTarget == HopperConstants.kJostleSetpoint)
          ? HopperConstants.kExtendedSetpoint
          : HopperConstants.kJostleSetpoint;
      m_hopper.jostleTo(m_jostleTarget);
    }
  }

  /** Ends jostling: clears the guard-bypass flag and parks the hopper fully extended. */
  public void endJostle() {
    m_jostling = false;
    m_hopper.jostleTo(HopperConstants.kExtendedSetpoint);
  }

  /**
   * Ensures the hopper is extended, then spins the gatherer at the default intake power.
   *
   * <p>Call repeatedly from a command's execute() loop.
   */
  public void possess() {
    if (!m_jostling && !m_hopper.isExtended()) {
      m_hopper.extend();
    }
    m_gather.gather(GatherConstants.kDefaultPower);
  }

  /**
   * Ensures the hopper is extended, then spins the gatherer at the given power.
   *
   * <p>Call repeatedly from a command's execute() loop.
   */
  public void possessWithPower(double power) {
    if (!m_jostling && !m_hopper.isExtended()) {
      m_hopper.extend();
    }
    m_gather.gather(power);
  }

  /** Stops gathering and optionally retracts. Gather stops; hopper stays in current position. */
  public void stop() {
    m_gather.rest();
  }
}
