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
 * <p>Uses the SparkFlex built-in motor encoder with a WPILib software PID controller for
 * closed-loop position control. Units are motor rotations (zeroed on startup).
 * The DIO Through Bore Encoder is kept initialized but unused.
 */
public class Hopper extends SubsystemBase {

  private final SparkFlex m_motor;
  private final Encoder m_throughBoreEncoder;
  private final RelativeEncoder m_encoder;
  private final PIDController m_pid;

  /** Current PID target (motor rotations). */
  private double m_setpoint = 0.0;
  /** Whether the software PID loop is active (disabled during testRun). */
  private boolean m_pidActive = false;

  /** Tracks whether the hopper is currently considered extended (for {@link #check()}). */
  private boolean m_extended = false;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testPositionPub;
  private final DoublePublisher m_testVelocityPub;
  private final DoublePublisher m_testSetpointPub;
  private final DoublePublisher m_testErrorPub;
  private final DoublePublisher m_testPPub;
  private final DoublePublisher m_testIPub;
  private final DoublePublisher m_testDPub;

  public Hopper() {
    m_motor = new SparkFlex(HopperConstants.kMotorId, MotorType.kBrushless);

    // Through Bore Encoder — kept initialized but no longer used for control
    m_throughBoreEncoder = new Encoder(HopperConstants.kEncoderDioA, HopperConstants.kEncoderDioB);

    m_pid = new PIDController(HopperConstants.kP, HopperConstants.kI, HopperConstants.kD);


    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);
    config.inverted(true);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);

    SmartDashboard.putNumber("Hopper PID/kP", HopperConstants.kP);
    SmartDashboard.putNumber("Hopper PID/kI", HopperConstants.kI);
    SmartDashboard.putNumber("Hopper PID/kD", HopperConstants.kD);

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Hopper");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testPositionPub = table.getDoubleTopic("Position").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
    m_testSetpointPub = table.getDoubleTopic("Setpoint").publish();
    m_testErrorPub = table.getDoubleTopic("Error").publish();
    m_testPPub = table.getDoubleTopic("kP").publish();
    m_testIPub = table.getDoubleTopic("kI").publish();
    m_testDPub = table.getDoubleTopic("kD").publish();

  }

  /** Extends the hopper fully to the configured setpoint. */
  public void extend() {
    m_setpoint = HopperConstants.kExtendedSetpoint;
    m_pidActive = true;
    m_extended = true;
  }

  /**
   * Extends the hopper to a percentage of its full travel.
   *
   * @param pct 0 = fully retracted, 100 = fully extended
   */
  public void extendTo(double pct) {
    m_setpoint = (pct / 100.0) * HopperConstants.kExtendedSetpoint;
    m_pidActive = true;
    m_extended = pct >= 95.0;
  }

  /** Retracts the hopper to its home position (encoder = 0). */
  public void retract() {
    m_setpoint = 0.0;
    m_pidActive = true;
    m_extended = false;
  }

  /**
   * Returns whether the hopper is currently extended.
   * Updated by {@link #extend()}, {@link #extendTo(double)}, and {@link #retract()}.
   */
  public boolean check() {
    return m_extended;
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
  }

  /** Enables or disables test mode telemetry publishing. */
  public void setTestMode(boolean enabled) {
    m_testMode = enabled;
  }

  @Override
  public void periodic() {
    m_pid.setP(SmartDashboard.getNumber("Hopper PID/kP", HopperConstants.kP));
    m_pid.setI(SmartDashboard.getNumber("Hopper PID/kI", HopperConstants.kI));
    m_pid.setD(SmartDashboard.getNumber("Hopper PID/kD", HopperConstants.kD));

    if (m_pidActive) {
      double output = m_pid.calculate(m_encoder.getPosition(), m_setpoint);
      m_motor.set(output);
    }
    if (m_testMode) {
      m_testPowerPub.set(m_motor.get());
      m_testPositionPub.set(m_encoder.getPosition());
      m_testVelocityPub.set(m_encoder.getVelocity());
      m_testSetpointPub.set(m_setpoint);
      m_testErrorPub.set(m_setpoint - m_encoder.getPosition());
      m_testPPub.set(m_pid.getP());
      m_testIPub.set(m_pid.getI());
      m_testDPub.set(m_pid.getD());
    }
  }
}
