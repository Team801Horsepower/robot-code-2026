// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AgitationType;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.SpindexConstants;

/**
 * Hopper – extends and retracts the hopper rail system, and can jostle in place.
 *
 * <p>Uses the built-in NEO Vortex relative encoder (acting like a quadrature encoder)
 * for closed-loop position control. The on-board REV V2 Absolute Encoder is physically
 * present but this code uses the relative encoder as specified ("relative encoder mode").
 * Zero the encoder at startup with the hopper fully retracted.
 *
 * <p>Amplitude constant shared with {@link Spindex} must be ≤ 0.5.
 */
public class Hopper extends SubsystemBase {

  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pid;

  /** Tracks whether the hopper is currently considered extended (for {@link #check()}). */
  private boolean m_extended = false;

  public Hopper() {
    m_motor = new SparkFlex(HopperConstants.kMotorId, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pid = m_motor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.closedLoop
        .p(HopperConstants.kP)
        .i(HopperConstants.kI)
        .d(HopperConstants.kD);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder.setPosition(0.0);
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
   * using the same agitation parameters as {@link Spindex#agitate()}.
   *
   * <p>Amplitude must be ≤ 0.5 (enforced by {@link SpindexConstants#kAmplitude}).
   * Call repeatedly (e.g. from a command's execute() loop).
   */
  public void jostle() {
    double amplitude = SpindexConstants.kAmplitude;
    double period    = SpindexConstants.kPeriod;
    boolean reversed = SpindexConstants.kAgitationReversed;
    double t = Timer.getFPGATimestamp();

    double maxPos = (1.0 - amplitude)        * HopperConstants.kExtendedSetpoint;
    double minPos = (1.0 - 2.0 * amplitude)  * HopperConstants.kExtendedSetpoint;
    double midPos = (maxPos + minPos) / 2.0;
    double half   = (maxPos - minPos) / 2.0;

    double setpoint;
    AgitationType type = SpindexConstants.kAgitationType;

    if (type == AgitationType.FLAT) {
      setpoint = reversed ? minPos : maxPos;

    } else if (type == AgitationType.SINUSOIDAL) {
      double sinVal = Math.sin(2.0 * Math.PI * t / period);
      setpoint = midPos + half * (reversed ? -sinVal : sinVal);

    } else { // ABSOLUTE_VALUE – triangle wave
      double phase = ((t % period) / period + 1.0) % 1.0;
      double tri   = 4.0 * Math.abs(phase - 0.5) - 1.0; // [-1, 1]
      setpoint = midPos + half * (reversed ? -tri : tri);
    }

    m_pid.setReference(setpoint, ControlType.kPosition);
  }
}
