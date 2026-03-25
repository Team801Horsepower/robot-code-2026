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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexConstants;

/**
 * Spindex – the center spindexer that stores game pieces and launches them toward the feeder.
 *
 * <p>Operating modes:
 * <ul>
 *   <li>{@link #spin()} – high-speed launch toward feeder via closed-loop velocity PID
 *   <li>{@link #rest()} – stop spinning
 * </ul>
 */
public class Spindex extends SubsystemBase {

  private final SparkFlex m_motor;
  private final SparkRelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pid;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testVelocityPub;

  private boolean reversing = false;

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
        .velocityFF(SpindexConstants.kVelocityFFkV);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pid = m_motor.getClosedLoopController();

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Spindex");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
    SmartDashboard.putNumber("SpindexerSpeed(RPM)", 0);
  }

  /** Spins the spindexer at launch velocity (toward feeder). */
  public void spin() {
    // m_pid.setReference(SpindexConstants.kTargetVelocityRPM, ControlType.kVelocity);
    m_pid.setReference(SmartDashboard.getNumber("SpindexerSpeed(RPM)", 0), ControlType.kVelocity);
  }

  /** Stops the spindexer. */
  public void rest() {
    if (reversing) {
      m_motor.set(-0.3);
    } else {
      m_motor.set(0.0);
    }
  }

  public void setReversing(boolean reversing) {
    this.reversing = reversing;
  }

  /** Drives the motor at raw power, bypassing PID. */
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
      m_testVelocityPub.set(m_encoder.getVelocity());
    }
  }
}
