// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * RunLaunchWheel – spins the turret launch wheels up to target velocity and holds them there.
 *
 * <p>This command never finishes on its own; cancel it or use it with {@code withTimeout()}.
 */
public class RunLaunchWheel extends Command {

  private final Turret m_turret;

  public RunLaunchWheel(Turret turret) {
    m_turret = turret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    m_turret.spin();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
