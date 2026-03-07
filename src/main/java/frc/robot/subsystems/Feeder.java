// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/**
 * Feeder – drives the series of spinners on the horizontal-to-vertical ramp
 * that carries game pieces from the spindexer into the turret.
 */
public class Feeder extends SubsystemBase {

  private final SparkFlex m_motor;

  public Feeder() {
    m_motor = new SparkFlex(FeederConstants.kMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Spins the feeder at the given power.
   *
   * @param power motor power in [-1, 1]; use positive for scoring direction
   */
  public void spin(double power) {
    m_motor.set(power);
  }

  /** Stops the feeder. */
  public void rest() {
    m_motor.set(0.0);
  }
}
