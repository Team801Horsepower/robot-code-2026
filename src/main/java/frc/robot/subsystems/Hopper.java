// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

/**
 * Hopper – extends and retracts the hopper rail system.
 *
 * <p>Uses the SparkFlex built-in motor encoder with separate WPILib software PID controllers
 * for extension and retraction. Units are motor rotations (zeroed on startup).
 * The DIO Through Bore Encoder is kept initialized but unused.
 */
public class Hopper extends SubsystemBase {

  private final SparkFlex m_motor;
  private final Encoder m_throughBoreEncoder;
  private final RelativeEncoder m_encoder;
  private final PIDController m_extendPid;
  private final PIDController m_retractPid;

  /** Current PID target (motor rotations). */
  private double m_setpoint = 0.0;
  /** Whether the software PID loop is active (disabled during testRun/stop). */
  private boolean m_pidActive = false;
  /** True when the active setpoint is an extension target, false for retraction. */
  private boolean m_extending = false;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testPositionPub;
  private final DoublePublisher m_testVelocityPub;
  private final DoublePublisher m_testSetpointPub;
  private final DoublePublisher m_testErrorPub;
  private final DoublePublisher m_testExtendPPub;
  private final DoublePublisher m_testExtendIPub;
  private final DoublePublisher m_testExtendDPub;
  private final DoublePublisher m_testRetractPPub;
  private final DoublePublisher m_testRetractIPub;
  private final DoublePublisher m_testRetractDPub;

  public Hopper() {
    m_motor = new SparkFlex(HopperConstants.kMotorId, MotorType.kBrushless);

    // Through Bore Encoder — kept initialized but no longer used for control
    m_throughBoreEncoder = new Encoder(HopperConstants.kEncoderDioA, HopperConstants.kEncoderDioB);

    m_extendPid = new PIDController(
        HopperConstants.kExtendP, HopperConstants.kExtendI, HopperConstants.kExtendD);
    m_retractPid = new PIDController(
        HopperConstants.kRetractP, HopperConstants.kRetractI, HopperConstants.kRetractD);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);
    config.inverted(true);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);

    // Publish PID controllers as Sendables for editing in Shuffleboard/AdvantageScope
    SmartDashboard.putData("Hopper Extend PID", m_extendPid);
    SmartDashboard.putData("Hopper Retract PID", m_retractPid);

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Hopper");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testPositionPub = table.getDoubleTopic("Position").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
    m_testSetpointPub = table.getDoubleTopic("Setpoint").publish();
    m_testErrorPub = table.getDoubleTopic("Error").publish();
    m_testExtendPPub = table.getDoubleTopic("Extend kP").publish();
    m_testExtendIPub = table.getDoubleTopic("Extend kI").publish();
    m_testExtendDPub = table.getDoubleTopic("Extend kD").publish();
    m_testRetractPPub = table.getDoubleTopic("Retract kP").publish();
    m_testRetractIPub = table.getDoubleTopic("Retract kI").publish();
    m_testRetractDPub = table.getDoubleTopic("Retract kD").publish();
  }

  /** Extends the hopper fully to the configured setpoint. */
  public void extend() {
    m_setpoint = HopperConstants.kExtendedSetpoint;
    m_pidActive = true;
    m_extending = true;
  }

  /**
   * Extends the hopper to a percentage of its full travel.
   *
   * @param pct 0 = fully retracted, 100 = fully extended
   */
  public void extendTo(double pct) {
    m_setpoint = (pct / 100.0) * HopperConstants.kExtendedSetpoint;
    m_pidActive = true;
    m_extending = true;
  }

  /** Retracts the hopper to its home position (encoder = 0). */
  public void retract() {
    m_setpoint = 2;
    m_pidActive = true;
    m_extending = false;
  }

  /** Stops the hopper motor immediately, disabling PID control. */
  public void stop() {
    m_pidActive = false;
    m_motor.stopMotor();
  }

  /**
   * Returns whether the hopper is currently at the extended position,
   * based on the actual encoder reading.
   */
  public boolean isExtended() {
    double pos = m_encoder.getPosition();
    return pos >= HopperConstants.kExtendMinPosition
        && pos <= HopperConstants.kExtendMaxPosition;
  }

  /**
   * Returns whether the hopper is currently at the retracted position,
   * based on the actual encoder reading.
   */
  public boolean isRetracted() {
    double pos = m_encoder.getPosition();
    return pos >= HopperConstants.kRetractMinPosition
        && pos <= HopperConstants.kRetractMaxPosition;
  }

  /**
   * Returns whether the hopper is currently extended.
   * @deprecated Use {@link #isExtended()} for position-based checking.
   */
  @Deprecated
  public boolean check() {
    return isExtended();
  }

  /** Drives the motor at raw power, bypassing PID control. */
  public void testRun(double power) {
    m_pidActive = false;
    m_motor.set(power);
  }

  /** Sets a PID position target for test mode tuning (motor rotations). */
  public void testSetPosition(double position) {
    m_setpoint = position;
    m_pidActive = true;
    m_extending = position > m_encoder.getPosition();
  }

  /** Enables or disables test mode telemetry publishing. */
  public void setTestMode(boolean enabled) {
    m_testMode = enabled;
  }

  @Override
  public void periodic() {
    if (m_pidActive) {
      PIDController activePid = m_extending ? m_extendPid : m_retractPid;
      double output = activePid.calculate(m_encoder.getPosition(), m_setpoint);
      m_motor.set(output);
    }

    SmartDashboard.putNumber("Hopper/Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Hopper Motor Power", m_motor.get());

    if (m_testMode) {
      m_testPowerPub.set(m_motor.get());
      m_testPositionPub.set(m_encoder.getPosition());
      m_testVelocityPub.set(m_encoder.getVelocity());
      m_testSetpointPub.set(m_setpoint);
      m_testErrorPub.set(m_setpoint - m_encoder.getPosition());
      m_testExtendPPub.set(m_extendPid.getP());
      m_testExtendIPub.set(m_extendPid.getI());
      m_testExtendDPub.set(m_extendPid.getD());
      m_testRetractPPub.set(m_retractPid.getP());
      m_testRetractIPub.set(m_retractPid.getI());
      m_testRetractDPub.set(m_retractPid.getD());
    }
  }
}
