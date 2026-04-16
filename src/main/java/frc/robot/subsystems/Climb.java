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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * Climb – drives the climb mechanism up and down via PID position control.
 *
 * <p>Uses the SparkFlex built-in motor encoder with a WPILib software PID controller.
 * Units are motor rotations (zeroed on startup). Idle mode is brake to prevent
 * the robot from descending when the motor is not actively driven.
 */
public class Climb extends SubsystemBase {

  private final SparkFlex m_motor;
  private final RelativeEncoder m_encoder;
  private final PIDController m_pid;

  /** Current PID target (motor rotations). */
  private double m_setpoint = 0.0;
  /** Whether the software PID loop is active (disabled during testRun/stop). */
  private boolean m_pidActive = false;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testPositionPub;
  private final DoublePublisher m_testVelocityPub;
  private final DoublePublisher m_testSetpointPub;
  private final DoublePublisher m_testErrorPub;
  private final DoublePublisher m_testPPub;
  private final DoublePublisher m_testIPub;
  private final DoublePublisher m_testDPub;

  public Climb() {
    m_motor = new SparkFlex(ClimbConstants.kMotorId, MotorType.kBrushless);

    m_pid = new PIDController(ClimbConstants.kResetP, ClimbConstants.kResetI, ClimbConstants.kResetD);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);

    SmartDashboard.putData("Climb PID", m_pid);

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Climb");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testPositionPub = table.getDoubleTopic("Position").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
    m_testSetpointPub = table.getDoubleTopic("Setpoint").publish();
    m_testErrorPub = table.getDoubleTopic("Error").publish();
    m_testPPub = table.getDoubleTopic("kP").publish();
    m_testIPub = table.getDoubleTopic("kI").publish();
    m_testDPub = table.getDoubleTopic("kD").publish();
  }

  public void extend() {
    m_setpoint = ClimbConstants.kExtendSetpoint;
    m_pidActive = true;
  }

  /** Returns the climb mechanism to the rest position (encoder = 0). */
  public void rest() {
    m_setpoint = 0.0;
    m_pidActive = true;
  }

  /** Stops the climb motor immediately, disabling PID control. */
  public void stop() {
    m_pidActive = false;
    m_motor.stopMotor();
  }

  /** Returns the current encoder position (motor rotations). */
  public double getPosition() {
    return m_encoder.getPosition();
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
    if (m_pidActive) {
      double output = m_pid.calculate(m_encoder.getPosition(), m_setpoint);
      m_motor.set(output);
    }

    SmartDashboard.putNumber("Climb/Position", m_encoder.getPosition());

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
