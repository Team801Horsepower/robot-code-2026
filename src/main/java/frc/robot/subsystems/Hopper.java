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


  private final SparkFlex m_motorR;
  private final SparkFlex m_motorL;
  private final Encoder m_throughBoreEncoder;
  private final RelativeEncoder m_encoderR;
  private final RelativeEncoder m_encoderL;
  private final PIDController m_extendRPid;
  private final PIDController m_retractRPid;
  private final PIDController m_extendLPid;
  private final PIDController m_retractLPid;

  /** Current PID target (motor rotations). */
  private double m_setpointR = 0.0;
  private double m_setpointL = 0.0;
  /** Whether the software PID loop is active (disabled during testRun/stop). */
  private boolean m_pidActiveR = false;
  private boolean m_pidActiveL = false;
  /** True when the active setpoint is an extension target, false for retraction. */
  private boolean m_extendingR = false;
  private boolean m_extendingL = false;

  private boolean m_testMode = false;
  private final DoublePublisher m_testPowerPub;
  private final DoublePublisher m_testPositionPub;
  private final DoublePublisher m_testVelocityPub;
  private final DoublePublisher m_testSetpointPub;
  private final DoublePublisher m_testErrorPub;
  private final DoublePublisher m_testExtendRPPub;
  private final DoublePublisher m_testExtendRIPub;
  private final DoublePublisher m_testExtendRDPub;
  private final DoublePublisher m_testRetractRPPub;
  private final DoublePublisher m_testRetractRIPub;
  private final DoublePublisher m_testRetractRDPub;
  private final DoublePublisher m_testExtendLPPub;
  private final DoublePublisher m_testExtendLIPub;
  private final DoublePublisher m_testExtendLDPub;
  private final DoublePublisher m_testRetractLPPub;
  private final DoublePublisher m_testRetractLIPub;
  private final DoublePublisher m_testRetractLDPub;

  public Hopper() {
    m_motorR = new SparkFlex(HopperConstants.kMotorRId, MotorType.kBrushless);
    m_motorL = new SparkFlex(HopperConstants.kMotorLId, MotorType.kBrushless);


    // Through Bore Encoder — kept initialized but no longer used for control
    m_throughBoreEncoder = new Encoder(HopperConstants.kEncoderDioA, HopperConstants.kEncoderDioB);


    m_extendRPid = new PIDController(
        HopperConstants.kExtendRP, HopperConstants.kExtendRI, HopperConstants.kExtendRD);
    m_retractRPid = new PIDController(
        HopperConstants.kRetractRP, HopperConstants.kRetractRI, HopperConstants.kRetractRD);
    m_extendLPid = new PIDController(
        HopperConstants.kExtendLP, HopperConstants.kExtendLI, HopperConstants.kExtendLD);
    m_retractLPid = new PIDController(
        HopperConstants.kRetractLP, HopperConstants.kRetractLI, HopperConstants.kRetractLD);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);
    config.inverted(true);


    m_motorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motorL.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_encoderR = m_motorR.getEncoder();
    m_encoderR.setPosition(0);
    m_encoderL = m_motorL.getEncoder();
    m_encoderL.setPosition(0);


    // Publish PID controllers as Sendables for editing in Shuffleboard/AdvantageScope
    SmartDashboard.putData("Hopper Right Extend PID", m_extendRPid);
    SmartDashboard.putData("Hopper Right Retract PID", m_retractRPid);
    SmartDashboard.putData("Hopper LEFT Extend PID", m_extendLPid);
    SmartDashboard.putData("Hopper LEFT Retract PID", m_retractLPid);

    var table = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Hopper");
    m_testPowerPub = table.getDoubleTopic("Power").publish();
    m_testPositionPub = table.getDoubleTopic("Position").publish();
    m_testVelocityPub = table.getDoubleTopic("VelocityRPM").publish();
    m_testSetpointPub = table.getDoubleTopic("Setpoint").publish();
    m_testErrorPub = table.getDoubleTopic("Error").publish();
    m_testExtendRPPub = table.getDoubleTopic("Right Extend kP").publish();
    m_testExtendRIPub = table.getDoubleTopic("Right Extend kI").publish();
    m_testExtendRDPub = table.getDoubleTopic("Right Extend kD").publish();
    m_testRetractRPPub = table.getDoubleTopic("Right Retract kP").publish();
    m_testRetractRIPub = table.getDoubleTopic("Right Retract kI").publish();
    m_testRetractRDPub = table.getDoubleTopic("Right Retract kD").publish();
    m_testExtendLPPub = table.getDoubleTopic("Left Extend kP").publish();
    m_testExtendLIPub = table.getDoubleTopic("Left Extend kI").publish();
    m_testExtendLDPub = table.getDoubleTopic("Left Extend kD").publish();
    m_testRetractLPPub = table.getDoubleTopic("Left Retract kP").publish();
    m_testRetractLIPub = table.getDoubleTopic("Left Retract kI").publish();
    m_testRetractLDPub = table.getDoubleTopic("Left Retract kD").publish();
  }


  /** Extends the hopper fully to the configured setpoint. */
  public void extend() {
    m_setpointR = HopperConstants.kExtendedSetpointR;
    m_setpointL = HopperConstants.kExtendedSetpointL;
    m_pidActiveR = true;
    m_pidActiveL = true;
    m_extendingR = true;
    m_extendingL = true;
  }


  /**
   * Extends the hopper to a percentage of its full travel.
   *
   * @param pct 0 = fully retracted, 100 = fully extended
   */
  public void extendTo(double pct) {
    m_setpointR = (pct / 100.0) * HopperConstants.kExtendedSetpointR;
    m_setpointL = (pct / 100.0) * HopperConstants.kExtendedSetpointL;
    m_pidActiveR = true;
    m_pidActiveL = true;
    m_extendingR = true;
    m_extendingL = true;
  }


  /** Retracts the hopper to its home position (encoder = 0). */
  public void retract() {
    m_setpointR = 0.5;
    m_setpointL = -0.5;
    m_pidActiveR = true;
    m_pidActiveL = true;
    m_extendingR = false;
    m_extendingL = false;
  }


  /** Drives to a jostle setpoint within the extended region, using the extend PID. */
  public void jostleTo(double position) {
    m_setpointR = position;
    m_setpointL = -position;
    m_pidActiveR = true;
    m_pidActiveL = true;
    m_extendingR = true; // reuse extendPid for both legs; both targets are above retract home
    m_extendingL = true;
  }


  /** True when encoder position is within tol of target (motor rotations). */
  public boolean isAt(double target, double tol) {
    return Math.abs(m_encoderR.getPosition() - target) <= tol;
  }


  /** Stops the hopper motor immediately, disabling PID control. */
  public void stop() {
    m_pidActiveR = false;
    m_pidActiveL = false;
    m_motorR.stopMotor();
    m_motorL.stopMotor();
  }


  /**
   * Returns whether the hopper is currently at the extended position,
   * based on the actual encoder reading.
   */
  public boolean isRightExtended() {
    double pos = m_encoderR.getPosition();
    return pos >= HopperConstants.kExtendMinPositionR
        && pos <= HopperConstants.kExtendMaxPositionR;
  }
  public boolean isLeftExtended() {
    double pos = m_encoderL.getPosition();
    return pos <= HopperConstants.kExtendMinPositionL
        && pos >= HopperConstants.kExtendMaxPositionL;
  }

  /**
   * Returns whether the hopper is currently at the retracted position,
   * based on the actual encoder reading.
   */
  public boolean isRightRetracted() {
    double pos = m_encoderR.getPosition();
    return pos >= HopperConstants.kRetractMinPositionR
        && pos <= HopperConstants.kRetractMaxPositionR;
  }
  public boolean isLeftRetracted() {
    double pos = m_encoderL.getPosition();
    return pos <= HopperConstants.kRetractMinPositionL
        && pos >= HopperConstants.kRetractMaxPositionL;
  }
  public boolean isExtended(){
    return isLeftExtended() && isRightExtended();
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
    m_pidActiveR = false;
    m_pidActiveL = false;
    m_motorR.set(power);
    m_motorL.set(-power);
  }


  /** Sets a PID position target for test mode tuning (motor rotations). */
  public void testSetPosition(double position) {
    m_setpointR = position;
    m_setpointL = -position;
    m_pidActiveR = true;
    m_pidActiveL = true;
    m_extendingR = position > m_encoderR.getPosition();
    m_extendingL = -position < m_encoderL.getPosition();
  }


  /** Enables or disables test mode telemetry publishing. */
  public void setTestMode(boolean enabled) {
    m_testMode = enabled;
  }


  @Override
  public void periodic() {
    if (m_pidActiveR) {
      PIDController activePid = m_extendingR ? m_extendRPid : m_retractRPid;
      double output = activePid.calculate(m_encoderR.getPosition(), m_setpointR);
      m_motorR.set(output);
    }
    if (m_pidActiveL) {
      PIDController activePid = m_extendingL ? m_extendLPid : m_retractLPid;
      double output = activePid.calculate(m_encoderL.getPosition(), m_setpointL);
      m_motorL.set(output);
    }

    SmartDashboard.putNumber("Rightside Hopper/Position", m_encoderR.getPosition());
    SmartDashboard.putNumber("Hopper Motor Right Power", m_motorR.get());
    SmartDashboard.putNumber("Leftside Hopper/Position", m_encoderL.getPosition());
    SmartDashboard.putNumber("Hopper Motor Left Power", m_motorL.get());


    if (m_testMode) {
      m_testPowerPub.set(m_motorR.get());
      m_testPowerPub.set(m_motorL.get());
      m_testPositionPub.set(m_encoderR.getPosition());
      m_testVelocityPub.set(m_encoderR.getVelocity());
      m_testPositionPub.set(m_encoderL.getPosition());
      m_testVelocityPub.set(m_encoderL.getVelocity());
      m_testSetpointPub.set(m_setpointR);
      m_testErrorPub.set(m_setpointR - m_encoderR.getPosition());
      m_testSetpointPub.set(m_setpointL);
      m_testErrorPub.set(m_setpointL - m_encoderL.getPosition());
      m_testExtendRPPub.set(m_extendRPid.getP());
      m_testExtendRIPub.set(m_extendRPid.getI());
      m_testExtendRDPub.set(m_extendRPid.getD());
      m_testRetractRPPub.set(m_retractRPid.getP());
      m_testRetractRIPub.set(m_retractRPid.getI());
      m_testRetractRDPub.set(m_retractRPid.getD());
      m_testExtendLPPub.set(m_extendLPid.getP());
      m_testExtendLIPub.set(m_extendLPid.getI());
      m_testExtendLDPub.set(m_extendLPid.getD());
      m_testRetractLPPub.set(m_retractLPid.getP());
      m_testRetractLIPub.set(m_retractLPid.getI());
      m_testRetractLDPub.set(m_retractLPid.getD());
    }
  }
}


