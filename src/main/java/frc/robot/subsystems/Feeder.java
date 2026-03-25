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
import frc.robot.Constants.FeederConstants;

/**
 * Feeder – drives the series of spinners on the horizontal-to-vertical ramp
 * that carries game pieces from the spindexer into the turret.
 */
public class Feeder extends SubsystemBase {

  private final SparkFlex m_motor;
  private final SparkRelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pid;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testVelocityPub;

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
        .velocityFF(FeederConstants.kVelocityFFkV);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pid = m_motor.getClosedLoopController();

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Feeder");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
    SmartDashboard.putNumber("FeederSpeed(RPM)", 0);
  }

  /** Spins the feeder at the target velocity. */
  public void spin() {
    m_pid.setReference(SmartDashboard.getNumber("FeederSpeed(RPM)", 0), ControlType.kVelocity);
  }

  /** Stops the feeder. */
  public void rest() {
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
    if (m_testMode) {
      m_testPowerPub.set(m_motor.get());
      m_testVelocityPub.set(m_encoder.getVelocity());
    }
  }
}
