// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Launch – composite that coordinates Spindex, Feeder, and Turret to score game pieces.
 *
 * <p>Calling {@link #launch()} spins all three in the correct sequence/power.
 */
public class Launch extends SubsystemBase {

  private final Spindex m_spindex;
  private final Feeder  m_feeder;
  private final Turret  m_turret;

  public Launch(Spindex spindex, Feeder feeder, Turret turret) {
    m_spindex = spindex;
    m_feeder  = feeder;
    m_turret  = turret;
  }

  /**
   * Executes the scoring sequence:
   * <ol>
   *   <li>Spins the spindexer at launch power.
   *   <li>Spins the feeder at full power.
   *   <li>Commands the turret to hold its current aim (keeps launch wheels spinning).
   * </ol>
   *
   * <p>Assumes {@link Turret#spin()} and {@link Turret#aim(double)} /
   * {@link Turret#rotate(double)} have already been called (e.g. via the Aim command).
   * Call repeatedly from a command's execute() loop.
   */
  public void launch() {
    m_spindex.spin();
    m_feeder.spin(1.0); // maximum positive power
    m_turret.spin();    // keep launch wheels at target velocity
  }

  /** Stops feeder and spindexer (turret wheels keep spinning intentionally). */
  public void stop() {
    m_spindex.rest();
    m_feeder.rest();
  }
}
