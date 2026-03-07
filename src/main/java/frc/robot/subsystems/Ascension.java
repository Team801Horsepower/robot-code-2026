// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Ascension – composite that manages the robot's climbing state.
 *
 * <p>Tracks the current climb level (0 = not climbed, 1 = L1, …) so descend()
 * knows which mechanism to actuate.
 */
public class Ascension extends SubsystemBase {

  private final Level1 m_level1;

  /** Current climb level: 0 = grounded, 1 = Level 1. */
  private int m_position = 0;

  public Ascension(Level1 level1) {
    m_level1 = level1;
  }

  /**
   * Initiates a Level 1 climb.
   *
   * <p>Sets internal position to 1 and starts driving the climber up.
   * Call repeatedly from a command's execute() loop until {@link Level1#isAtSetpoint()}.
   */
  public void levelOne() {
    m_position = 1;
    m_level1.ascend();
  }

  /** Placeholder for a future Level 3 climb. No-op for now. */
  public void levelThree() {
    // TODO: implement when L3 mechanism is designed
  }

  /**
   * Descends from the current climb level.
   *
   * <p>Currently only L1 descent is supported. Call repeatedly until
   * {@link Level1#isRetracted()}.
   */
  public void descend() {
    if (m_position == 1) {
      m_level1.descend();
      if (m_level1.isRetracted()) {
        m_position = 0;
      }
    }
    // TODO: handle m_position > 1 when higher levels are implemented
  }

  /** Returns true when the Level 1 climber has physically reached its extended setpoint. */
  public boolean isAtSetpoint() {
    return m_level1.isAtSetpoint();
  }

  /** Returns the current climb position (0 = grounded, 1 = L1). */
  public int getPosition() {
    return m_position;
  }
}
