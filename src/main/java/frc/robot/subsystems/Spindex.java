// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SparkClosedLoopController m_pid;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testVelocityPub;

  private AgitationType m_currentAgitationType = SpindexConstants.kAgitationType;

  private SpinState m_spinState       = SpinState.SPINNING;
  private double    m_spinStartTime   = -1.0; // -1 = not currently spinning
  private double    m_jamReverseStart = 0.0;

  // Live-tunable PIDF gains (read from SmartDashboard each cycle)
  private double m_lastP  = SpindexConstants.kVelocityP;
  private double m_lastI  = SpindexConstants.kVelocityI;
  private double m_lastD  = SpindexConstants.kVelocityD;
  private double m_lastFF = SpindexConstants.kVelocityFF;

  public Spindex() {
    m_motor = new SparkFlex(SpindexConstants.kMotorId, MotorType.kBrushless);
    m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);
    config.inverted(true);
    config.closedLoop
        .p(SpindexConstants.kVelocityP)
        .i(SpindexConstants.kVelocityI)
        .d(SpindexConstants.kVelocityD)
        .velocityFF(SpindexConstants.kVelocityFF);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pid = m_motor.getClosedLoopController();

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Spindex");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();

    // Publish initial PIDF values (editable in SmartDashboard/Shuffleboard)
    SmartDashboard.putNumber("Spindex/P", m_lastP);
    SmartDashboard.putNumber("Spindex/I", m_lastI);
    SmartDashboard.putNumber("Spindex/D", m_lastD);
    SmartDashboard.putNumber("Spindex/FF", m_lastFF);
  }

  /** Spins the spindexer at launch power (negative = toward feeder). */
  public void spin() {
    spin(false);
  }

  /** Spins the spindexer at launch power. When complexMode is true, jam detection is active. */
  public void spin(boolean complexMode) {
    double now = Timer.getFPGATimestamp();

    // Record when spin() was first called after a rest()
    if (m_spinStartTime < 0.0) {
      m_spinStartTime = now;
    }

    if (complexMode) {
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
    } else {
      // No jam detection — reset state in case we just left complex mode
      m_spinState = SpinState.SPINNING;
    }

    m_pid.setReference(SpindexConstants.kTargetVelocityRPM, ControlType.kVelocity);
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

  /** Drives the motor at raw power, bypassing jam detection. */
  public void testRun(double power) {
    m_motor.set(power);
  }

  /** Enables or disables test mode telemetry publishing. */
  public void setTestMode(boolean enabled) {
    m_testMode = enabled;
  }

  @Override
  public void periodic() {
    // Read PIDF values from SmartDashboard (editable at runtime)
    double p  = SmartDashboard.getNumber("Spindex/P", m_lastP);
    double i  = SmartDashboard.getNumber("Spindex/I", m_lastI);
    double d  = SmartDashboard.getNumber("Spindex/D", m_lastD);
    double ff = SmartDashboard.getNumber("Spindex/FF", m_lastFF);

    // Reconfigure PID only when values change
    if (p != m_lastP || i != m_lastI || d != m_lastD || ff != m_lastFF) {
      SparkFlexConfig config = new SparkFlexConfig();
      config.closedLoop.p(p).i(i).d(d).velocityFF(ff);
      m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      m_lastP = p;
      m_lastI = i;
      m_lastD = d;
      m_lastFF = ff;
    }

    if (m_testMode) {
      m_testPowerPub.set(m_motor.get());
      m_testVelocityPub.set(m_encoder.getVelocity());
    }
  }
}
