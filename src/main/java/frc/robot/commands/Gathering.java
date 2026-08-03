// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gather;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Possession;

/**
 * Gathering – continuously runs the intake sequence (extend hopper + spin gatherer).
 *
 * <p>This command runs indefinitely; cancel it when the driver releases the button.
 */
public class Gathering extends Command {

  private final Possession m_possession;
  private final DoubleSupplier m_powerSupplier;

  public Gathering(Possession possession, Gather gather, Hopper hopper,
      DoubleSupplier powerSupplier) {
    m_possession = possession;
    m_powerSupplier = powerSupplier;
    addRequirements(possession, gather, hopper);
  }

  @Override
  public void execute() {
    m_possession.possessWithPower();
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
