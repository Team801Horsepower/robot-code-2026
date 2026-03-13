// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Level1 – controls the two-stage climber slide.
 *
 * <p>The climber uses a single NEO Vortex with the built-in relative encoder for position
 * tracking. An {@link ElevatorFeedforward} holds the arm at the extended setpoint against
 * gravity once reached.
 *
 * <p>Important: zero the encoder before use (robot starts with climber fully retracted).
 */
public class Level1 extends SubsystemBase {

  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testPositionPub;

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
      ClimberConstants.kFeedforwardKs,
      ClimberConstants.kFeedforwardKg,
      ClimberConstants.kFeedforwardKv);

  public Level1() {
    m_motor  = new SparkFlex(ClimberConstants.kMotorId, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(60);
    config.inverted(true);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder.setPosition(0.0);

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Level1");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testPositionPub = table.getDoubleTopic("PositionRotations").publish();
  }

  /**
   * Extends the climber to {@link ClimberConstants#kExtendedSetpoint}.
   *
   * <p>While below the setpoint, drives the motor at {@link ClimberConstants#kMotorPower}.
   * Once at the setpoint, switches to feedforward voltage to hold against gravity.
   * Call repeatedly from a command's execute() loop.
   */
  public void ascend() {
    double pos = m_encoder.getPosition();
    if (pos < ClimberConstants.kExtendedSetpoint - ClimberConstants.kTolerance) {
      m_motor.set(ClimberConstants.kMotorPower);
    } else {
      // At setpoint: apply gravity-compensation feedforward (velocity = 0).
      double holdVolts = m_feedforward.calculate(0.0);
      m_motor.setVoltage(holdVolts);
    }
  }

  /**
   * Retracts the climber back to home (encoder ≈ 0).
   * Call repeatedly from a command's execute() loop.
   */
  public void descend() {
    double pos = m_encoder.getPosition();
    if (pos > ClimberConstants.kTolerance) {
      m_motor.set(-ClimberConstants.kMotorPower);
    } else {
      m_motor.set(0.0);
    }
  }

  /** Returns whether the climber is at the extended setpoint. */
  public boolean isAtSetpoint() {
    return Math.abs(m_encoder.getPosition() - ClimberConstants.kExtendedSetpoint)
        <= ClimberConstants.kTolerance;
  }

  /** Returns whether the climber is fully retracted (at home). */
  public boolean isRetracted() {
    return m_encoder.getPosition() <= ClimberConstants.kTolerance;
  }

  /** Drives the motor at raw power, bypassing feedforward control. */
  public void testRun(double power) {
    m_motor.set(power);
  }

  /** Enables or disables test mode telemetry publishing. */
  public void setTestMode(boolean enabled) {
    m_testMode = enabled;
  }

  @Override
  public void periodic() {
    if (m_testMode) {
      m_testPowerPub.set(m_motor.get());
      m_testPositionPub.set(m_encoder.getPosition());
    }
  }
}
