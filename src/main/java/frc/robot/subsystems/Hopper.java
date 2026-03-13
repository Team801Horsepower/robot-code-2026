// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AgitationType;
import frc.robot.Constants.HopperConstants;

/**
 * Hopper – extends and retracts the hopper rail system, and can jostle in place.
 *
 * <p>Uses the REV Through Bore Encoder (via SparkFlex data port, relative/quadrature mode) with a
 * 2.1 in/rotation conversion factor for closed-loop position control. Zero reference is the
 * physical home/retracted position (encoder zeroes on power-on with hopper retracted).
 */
public class Hopper extends SubsystemBase {

  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pid;

  /** Tracks whether the hopper is currently considered extended (for {@link #check()}). */
  private boolean m_extended = false;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testPositionPub;
  private final DoublePublisher m_testVelocityPub;

  public Hopper() {
    m_motor = new SparkFlex(HopperConstants.kMotorId, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pid = m_motor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(60);
    config.inverted(true);
    config.encoder.positionConversionFactor(HopperConstants.kEncoderConversionFactor);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(HopperConstants.kP)
        .i(HopperConstants.kI)
        .d(HopperConstants.kD);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Hopper");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testPositionPub = table.getDoubleTopic("PositionInches").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
  }

  /** Extends the hopper fully to the configured setpoint. */
  public void extend() {
    m_pid.setReference(HopperConstants.kExtendedSetpoint, ControlType.kPosition);
    m_extended = true;
  }

  /**
   * Extends the hopper to a percentage of its full travel.
   *
   * @param pct 0 = fully retracted, 100 = fully extended
   */
  public void extendTo(double pct) {
    double setpoint = (pct / 100.0) * HopperConstants.kExtendedSetpoint;
    m_pid.setReference(setpoint, ControlType.kPosition);
    m_extended = pct >= 95.0;
  }

  /** Retracts the hopper to its home position (encoder = 0). */
  public void retract() {
    m_pid.setReference(0.0, ControlType.kPosition);
    m_extended = false;
  }

  /**
   * Returns whether the hopper is currently extended.
   * Updated by {@link #extend()}, {@link #extendTo(double)}, and {@link #retract()}.
   */
  public boolean check() {
    return m_extended;
  }

  /**
   * Oscillates the hopper between positions (1−amplitude)·setpoint and (1−2·amplitude)·setpoint
   * using independent {@link HopperConstants} jostle constants.
   *
   * <p>Call repeatedly (e.g. from a command's execute() loop).
   */
  public void jostle() {
    double amplitude = HopperConstants.kJostleAmplitude;
    double period    = HopperConstants.kJostlePeriod;
    double t = Timer.getFPGATimestamp();

    double maxPos = (1.0 - amplitude)        * HopperConstants.kExtendedSetpoint;
    double minPos = (1.0 - 2.0 * amplitude)  * HopperConstants.kExtendedSetpoint;
    double midPos = (maxPos + minPos) / 2.0;
    double half   = (maxPos - minPos) / 2.0;

    double setpoint;
    AgitationType type = HopperConstants.kJostleAgitationType;

    if (type == AgitationType.FLAT) {
      setpoint = maxPos;

    } else if (type == AgitationType.SINUSOIDAL) {
      setpoint = midPos + half * Math.sin(2.0 * Math.PI * t / period);

    } else { // ABSOLUTE_VALUE – triangle wave
      double phase = ((t % period) / period + 1.0) % 1.0;
      double tri   = 4.0 * Math.abs(phase - 0.5) - 1.0; // [-1, 1]
      setpoint = midPos + half * tri;
    }

    m_pid.setReference(setpoint, ControlType.kPosition);
  }

  /** Drives the motor at raw power, bypassing PID control. */
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
      m_testVelocityPub.set(m_encoder.getVelocity());
    }
  }
}
