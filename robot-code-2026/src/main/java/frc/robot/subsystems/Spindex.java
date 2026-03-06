// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AgitationType;
import frc.robot.Constants.SpindexConstants;

/**
 * Spindex – the center spindexer that stores game pieces and launches them toward the feeder.
 *
 * <p>Three operating modes:
 * <ul>
 *   <li>{@link #spin()} – constant high-speed launch toward feeder (negative direction)
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

  private final SparkFlex m_motor;

  public Spindex() {
    m_motor = new SparkFlex(SpindexConstants.kMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Spins the spindexer at launch power (negative = toward feeder). */
  public void spin() {
    m_motor.set(-SpindexConstants.kSpinPower);
  }

  /** Stops the spindexer. */
  public void rest() {
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
    AgitationType type = SpindexConstants.kAgitationType;

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
}
