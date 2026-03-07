// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AgitationType;
import frc.robot.Constants.SpindexConstants;

/**
 * Spindex – the center spindexer that stores game pieces and launches them toward the feeder.
 *
 * <p>Three operating modes:
 * <ul>
 *   <li>{@link #spin()} – high-speed launch toward feeder; detects jams via encoder velocity
 *       and auto-reverses to clear them. Prints a DS warning on each jam event.
 *   <li>{@link #rest()} – stop spinning
 *   <li>{@link #agitate()} – gently oscillate to prevent jamming; call repeatedly from
 *       command execute()
 * </ul>
 *
 * <p>Agitation waveforms (controlled by {@link SpindexConstants}):
 * <ul>
 *   <li>FLAT – constant power at ±amplitude
 *   <li>SINUSOIDAL – smooth sine wave
 *   <li>ABSOLUTE_VALUE – triangle wave
 * </ul>
 *
 * <p>Reversed=false → minimum power is 0 (center at +amplitude, range [0, 2·amplitude]).<br>
 * Reversed=true  → zero is center (range [−amplitude, +amplitude]).
 */
public class Spindex extends SubsystemBase {

  /** Tracks the current phase of spin() operation. */
  private enum SpinState { SPINNING, JAM_REVERSING }

  private final SparkFlex m_motor;
  private final SparkRelativeEncoder m_encoder;

  private AgitationType m_currentAgitationType = SpindexConstants.kAgitationType;

  private SpinState m_spinState       = SpinState.SPINNING;
  private double    m_spinStartTime   = -1.0; // -1 = not currently spinning
  private double    m_jamReverseStart = 0.0;

  public Spindex() {
    m_motor = new SparkFlex(SpindexConstants.kMotorId, MotorType.kBrushless);
    m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spins the spindexer at launch power (negative = toward feeder). */
  public void spin() {
    double now = Timer.getFPGATimestamp();

    // Record when spin() was first called after a rest()
    if (m_spinStartTime < 0.0) {
      m_spinStartTime = now;
    }

    // If currently reversing to clear a jam, keep reversing until time is up
    if (m_spinState == SpinState.JAM_REVERSING) {
      if (now - m_jamReverseStart < SpindexConstants.kJamReverseTime) {
        m_motor.set(SpindexConstants.kJamReversePower);
        return;
      }
      // Reverse complete — resume spinning
      m_spinState = SpinState.SPINNING;
    }

    // Check for a jam (only after debounce window to allow motor spin-up)
    double velocityRPM = m_encoder.getVelocity();
    if (now - m_spinStartTime > SpindexConstants.kJamDetectDebounce
        && Math.abs(velocityRPM) < SpindexConstants.kJamVelocityThresholdRPM) {
      m_spinState       = SpinState.JAM_REVERSING;
      m_jamReverseStart = now;
      DriverStation.reportWarning(
          "Spindex jam detected (velocity=" + velocityRPM + " RPM)", false);
      m_motor.set(SpindexConstants.kJamReversePower);
      return;
    }

    m_motor.set(-SpindexConstants.kSpinPower);
  }

  /** Stops the spindexer. */
  public void rest() {
    m_spinState     = SpinState.SPINNING;
    m_spinStartTime = -1.0;
    m_motor.set(0.0);
  }

  /**
   * Applies a time-varying agitation waveform to prevent game-piece jams.
   * Must be called repeatedly (e.g. from a command's execute() loop).
   *
   * <p>Waveform is determined by {@link SpindexConstants#kAgitationType},
   * {@link SpindexConstants#kAmplitude}, {@link SpindexConstants#kPeriod},
   * and {@link SpindexConstants#kAgitationReversed}.
   */
  public void agitate() {
    double amplitude = SpindexConstants.kAmplitude;
    double period    = SpindexConstants.kPeriod;
    boolean reversed = SpindexConstants.kAgitationReversed;
    double t = Timer.getFPGATimestamp();

    double power;
    AgitationType type = m_currentAgitationType;

    if (type == AgitationType.FLAT) {
      // Constant power; sign determined by reversed flag.
      power = reversed ? -amplitude : amplitude;

    } else if (type == AgitationType.SINUSOIDAL) {
      double sinVal = Math.sin(2.0 * Math.PI * t / period);
      if (reversed) {
        // Oscillates from -amplitude to +amplitude, centered at 0.
        power = amplitude * sinVal;
      } else {
        // Oscillates from 0 to 2·amplitude, centered at +amplitude.
        power = amplitude + amplitude * sinVal;
      }

    } else { // ABSOLUTE_VALUE – triangle wave
      // Triangle: +1 at t=0, period, 2·period; -1 at t=period/2, 3·period/2, ...
      double phase = ((t % period) / period + 1.0) % 1.0;
      double tri   = 4.0 * Math.abs(phase - 0.5) - 1.0; // [-1, 1]
      if (reversed) {
        power = amplitude * tri;
      } else {
        power = amplitude + amplitude * tri;
      }
    }

    m_motor.set(power);
  }

  /** Advances to the next agitation type in the cycle: FLAT → SINUSOIDAL → ABSOLUTE_VALUE → FLAT. */
  public void cycleAgitationType() {
    AgitationType[] types = AgitationType.values();
    m_currentAgitationType = types[(m_currentAgitationType.ordinal() + 1) % types.length];
  }
}
