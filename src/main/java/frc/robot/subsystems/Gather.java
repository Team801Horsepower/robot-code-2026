// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GatherConstants;

/**
 * Gather – controls the roller at the end of the hopper that pulls game pieces in.
 *
 * <p>Simple open-loop power control. The hopper must be extended before gathering
 * (enforced in {@link Possession}).
 */
public class Gather extends SubsystemBase {

  private final SparkFlex m_motor;

  public Gather() {
    m_motor = new SparkFlex(GatherConstants.kMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spins the gatherer roller at the given power. Positive = intake direction. */
  public void gather(double power) {
    m_motor.set(power);
  }

  /** Stops the gatherer roller. */
  public void rest() {
    m_motor.set(0.0);
  }
}
