// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manipulator – top-level composite that orchestrates Possession and Launch for scoring.
 *
 * <p>Primarily used from autonomous commands via {@link #score()}.
 */
public class Manipulator extends SubsystemBase {

  private final Possession m_possession;
  private final Launch     m_launch;

  public Manipulator(Possession possession, Launch launch) {
    m_possession = possession;
    m_launch     = launch;
  }

  /**
   * Runs the full score cycle: intake if not already possessing, then launch.
   *
   * <p>Calls {@link Possession#possess()} and {@link Launch#launch()} together.
   * Call repeatedly from a command's execute() loop.
   */
  public void score() {
    m_possession.possess();
    m_launch.launch();
  }

  /** Stops all scoring mechanisms. */
  public void stop() {
    m_possession.stop();
    m_launch.stop();
  }
}
