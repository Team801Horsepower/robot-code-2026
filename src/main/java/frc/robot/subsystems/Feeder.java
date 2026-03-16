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
import frc.robot.Constants.FeederConstants;

/**
 * Feeder – drives the series of spinners on the horizontal-to-vertical ramp
 * that carries game pieces from the spindexer into the turret.
 *
 * <p>Includes velocity-based jam detection that mirrors the spindexer's
 * implementation: when encoder RPM drops below a threshold after a debounce
 * window, the motor auto-reverses to clear the jam.
 */
public class Feeder extends SubsystemBase {

  /** Tracks the current phase of spin() operation. */
  private enum SpinState { SPINNING, JAM_REVERSING }

  private final SparkFlex m_motor;
  private final SparkRelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pid;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testVelocityPub;

  private SpinState m_spinState       = SpinState.SPINNING;
  private double    m_spinStartTime   = -1.0; // -1 = not currently spinning
  private double    m_jamReverseStart = 0.0;

  public Feeder() {
    m_motor = new SparkFlex(FeederConstants.kMotorId, MotorType.kBrushless);
    m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);
    config.closedLoop
        .p(FeederConstants.kVelocityP)
        .i(FeederConstants.kVelocityI)
        .d(FeederConstants.kVelocityD)
        .velocityFF(FeederConstants.kVelocityFF);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pid = m_motor.getClosedLoopController();

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Feeder");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
  }

  /**
   * Spins the feeder at the target velocity, with jam detection.
   *
   * <p>After a debounce window, if encoder velocity drops below the jam
   * threshold the motor reverses automatically for a fixed duration to
   * clear the jam, then resumes normal spinning.
   */
  public void spin() {
    double now = Timer.getFPGATimestamp();

    // Record when spin() was first called after a rest()
    if (m_spinStartTime < 0.0) {
      m_spinStartTime = now;
    }

    // If currently reversing to clear a jam, keep reversing until time is up
    if (m_spinState == SpinState.JAM_REVERSING) {
      if (now - m_jamReverseStart < FeederConstants.kJamReverseTime) {
        m_motor.set(-FeederConstants.kJamReversePower);
        return;
      }
      // Reverse complete — resume spinning
      m_spinState = SpinState.SPINNING;
    }

    // Check for a jam (only after debounce window to allow motor spin-up)
    double velocityRPM = m_encoder.getVelocity();
    if (now - m_spinStartTime > FeederConstants.kJamDetectDebounce
        && Math.abs(velocityRPM) < FeederConstants.kJamVelocityThresholdRPM) {
      m_spinState       = SpinState.JAM_REVERSING;
      m_jamReverseStart = now;
      DriverStation.reportWarning(
          "Feeder jam detected (velocity=" + velocityRPM + " RPM)", false);
      m_motor.set(-FeederConstants.kJamReversePower);
      return;
    }

    m_pid.setReference(FeederConstants.kTargetVelocityRPM, ControlType.kVelocity);
  }

  /** Stops the feeder and resets jam detection state. */
  public void rest() {
    m_spinState     = SpinState.SPINNING;
    m_spinStartTime = -1.0;
    m_motor.set(0.0);
  }

  /** Drives the motor at the given power for test mode. */
  public void testRun(double power) {
    m_motor.set(power);
  }

  /** Enables or disables test-mode telemetry publishing. */
  public void setTestMode(boolean enabled) {
    m_testMode = enabled;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder/P", FeederConstants.kVelocityP);
    SmartDashboard.putNumber("Feeder/I", FeederConstants.kVelocityI);
    SmartDashboard.putNumber("Feeder/D", FeederConstants.kVelocityD);
    SmartDashboard.putNumber("Feeder/FF", FeederConstants.kVelocityFF);

    if (m_testMode) {
      m_testPowerPub.set(m_motor.get());
      m_testVelocityPub.set(m_encoder.getVelocity());
    }
  }
}
