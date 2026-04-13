// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Rumbles the controller after it has been scheduled for a specified duration.
 *
 * <p>Designed to run in parallel with another command (e.g., {@link Shoot}) to
 * alert the driver that the trigger has been held for an extended period.
 */
public class Rumble extends Command {

  private final CommandXboxController m_controller;
  private final double m_delaySec;
  private final Timer m_timer = new Timer();

  /**
   * @param controller the driver controller to rumble
   * @param delaySec   seconds to wait before rumbling
   */
  public Rumble(CommandXboxController controller, double delaySec) {
    m_controller = controller;
    m_delaySec = delaySec;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(m_delaySec)) {
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
