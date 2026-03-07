// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GatherConstants;

/**
 * Possession – composite that coordinates the Hopper and Gather subsystems to intake game pieces.
 *
 * <p>The hopper must be extended before the gatherer can run. {@link #possess()} enforces this.
 */
public class Possession extends SubsystemBase {

  private final Hopper m_hopper;
  private final Gather m_gather;

  public Possession(Hopper hopper, Gather gather) {
    m_hopper = hopper;
    m_gather = gather;
  }

  /**
   * Ensures the hopper is extended, then spins the gatherer at the default intake power.
   *
   * <p>Call repeatedly from a command's execute() loop.
   */
  public void possess() {
    if (!m_hopper.check()) {
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
    m_hopper.extend();
    m_gather.gather(power);
  }

  /** Stops gathering and optionally retracts. Gather stops; hopper stays in current position. */
  public void stop() {
    m_gather.rest();
  }
}
