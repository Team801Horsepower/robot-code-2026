// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Turret – the scoring mechanism housing the launch wheels, hood, and rotation stage.
 *
 * <h2>Sub-systems</h2>
 * <ul>
 *   <li><b>Launch wheels</b> – two NEO Vortex motors running in opposite directions, closed-loop
 *       velocity targeting {@link TurretConstants#kTargetVelocityRPM}.
 *   <li><b>Hood</b> – one NEO Vortex motor with built-in relative encoder; adjusts launch angle
 *       15–30°.
 *   <li><b>Rotate</b> – one NEO Vortex motor with a REV V2 Absolute Encoder (conversion factor
 *       maps one full encoder rotation to 420° of turret travel); ±210° range.
 * </ul>
 *
 * <p>Note: the turret spins throughout the entire match; there is no rest() for the launch wheels.
 */
public class Turret extends SubsystemBase {

  // ─── Launch wheels ─────────────────────────────────────────────────────────

  private final SparkFlex m_launchMotor1;
  private final SparkFlex m_launchMotor2;
  private final SparkClosedLoopController m_launchPid;

  // ─── Hood ──────────────────────────────────────────────────────────────────

  private final SparkFlex m_hoodMotor;
  private final RelativeEncoder m_hoodEncoder;

  // ─── Rotation ──────────────────────────────────────────────────────────────

  private final SparkFlex m_rotateMotor;
  /** V2 Absolute Encoder configured with 420°/revolution conversion factor. */
  private final SparkAbsoluteEncoder m_rotateEncoder;

  private boolean m_testMode = false;
  private final DoublePublisher m_testLaunch1PowerPub;
  private final DoublePublisher m_testLaunch1VelocityPub;
  private final DoublePublisher m_testLaunch2PowerPub;
  private final DoublePublisher m_testLaunch2VelocityPub;
  private final DoublePublisher m_testHoodPowerPub;
  private final DoublePublisher m_testHoodPositionPub;
  private final DoublePublisher m_testRotatePowerPub;
  private final DoublePublisher m_testRotatePositionPub;

  public Turret() {
    // ── Launch motors ─────────────────────────────────────────────────────────
    m_launchMotor1 = new SparkFlex(TurretConstants.kLaunchMotor1Id, MotorType.kBrushless);
    m_launchMotor2 = new SparkFlex(TurretConstants.kLaunchMotor2Id, MotorType.kBrushless);
    m_launchPid    = m_launchMotor1.getClosedLoopController();

    SparkFlexConfig launchConfig1 = new SparkFlexConfig();
    launchConfig1.idleMode(IdleMode.kCoast);
    launchConfig1.smartCurrentLimit(60);
    launchConfig1.inverted(true);
    launchConfig1.closedLoop
        .p(TurretConstants.kVelocityP)
        .velocityFF(TurretConstants.kVelocityFF);
    m_launchMotor1.configure(launchConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Motor 2 mirrors motor 1 (physically facing the opposite direction).
    SparkFlexConfig launchConfig2 = new SparkFlexConfig();
    launchConfig2.idleMode(IdleMode.kCoast);
    launchConfig2.smartCurrentLimit(60);
    launchConfig2.inverted(true);
    m_launchMotor2.configure(launchConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ── Hood motor ────────────────────────────────────────────────────────────
    m_hoodMotor   = new SparkFlex(TurretConstants.kHoodMotorId, MotorType.kBrushless);
    m_hoodEncoder = m_hoodMotor.getEncoder();

    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.smartCurrentLimit(60);
    hoodConfig.inverted(true);
    m_hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_hoodEncoder.setPosition(0.0); // physical position at boot = 15° minimum

    // ── Rotate motor ──────────────────────────────────────────────────────────
    m_rotateMotor   = new SparkFlex(TurretConstants.kRotateMotorId, MotorType.kBrushless);
    m_rotateEncoder = m_rotateMotor.getAbsoluteEncoder();

    SparkFlexConfig rotateConfig = new SparkFlexConfig();
    rotateConfig.idleMode(IdleMode.kBrake);
    rotateConfig.smartCurrentLimit(60);
    rotateConfig.inverted(true);
    // One full encoder revolution = 420° of turret travel.
    rotateConfig.absoluteEncoder.positionConversionFactor(TurretConstants.kRotateEncoderConversionFactor);
    m_rotateMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var turretTable = NetworkTableInstance.getDefault().getTable("TestMode").getSubTable("Turret");
    m_testLaunch1PowerPub    = turretTable.getSubTable("Launch1").getDoubleTopic("Power").publish();
    m_testLaunch1VelocityPub = turretTable.getSubTable("Launch1").getDoubleTopic("VelocityRPM").publish();
    m_testLaunch2PowerPub    = turretTable.getSubTable("Launch2").getDoubleTopic("Power").publish();
    m_testLaunch2VelocityPub = turretTable.getSubTable("Launch2").getDoubleTopic("VelocityRPM").publish();
    m_testHoodPowerPub       = turretTable.getSubTable("Hood").getDoubleTopic("Power").publish();
    m_testHoodPositionPub    = turretTable.getSubTable("Hood").getDoubleTopic("Position").publish();
    m_testRotatePowerPub     = turretTable.getSubTable("Rotate").getDoubleTopic("Power").publish();
    m_testRotatePositionPub  = turretTable.getSubTable("Rotate").getDoubleTopic("PositionDeg").publish();
  }

  /**
   * Runs the launch wheels at the target velocity. Both motors follow motor 1's closed-loop
   * output; motor 2 is inverted to spin in the correct direction.
   *
   * <p>Call once to start; wheels keep spinning until power is cut.
   */
  public void spin() {
    m_launchPid.setReference(TurretConstants.kTargetVelocityRPM, ControlType.kVelocity);
    // Mirror motor 2 to motor 1's output (they spin in opposite directions physically).
    m_launchMotor2.set(m_launchMotor1.getAppliedOutput());
  }

  /**
   * Adjusts the hood angle.
   *
   * <p>The hood encoder starts at 0 when the hood is at the minimum angle
   * ({@link TurretConstants#kHoodMinDeg}). One encoder rotation corresponds to some physical
   * angle change – tune {@link TurretConstants#kHoodMaxEncoderPos} on the robot.
   *
   * @param angleDegrees target hood angle, clamped to [kHoodMinDeg, kHoodMaxDeg]
   */
  public void aim(double angleDegrees) {
    angleDegrees = Math.max(TurretConstants.kHoodMinDeg,
                   Math.min(TurretConstants.kHoodMaxDeg, angleDegrees));

    // Map [kHoodMinDeg, kHoodMaxDeg] → [0, kHoodMaxEncoderPos].
    double targetPos = (angleDegrees - TurretConstants.kHoodMinDeg)
        / (TurretConstants.kHoodMaxDeg - TurretConstants.kHoodMinDeg)
        * TurretConstants.kHoodMaxEncoderPos;

    double currentPos = m_hoodEncoder.getPosition();
    double error = targetPos - currentPos;

    if (Math.abs(error) > TurretConstants.kAimToleranceEncoder) {
      double power = TurretConstants.kAimPower * Math.signum(error);
      // Hardware safety: don't overrun physical limits.
      if (currentPos <= 0 && power < 0) power = 0;
      if (currentPos >= TurretConstants.kHoodPhysicalMaxEncoderPos && power > 0) power = 0;
      m_hoodMotor.set(power);
    } else {
      m_hoodMotor.set(0.0);
    }
  }

  /**
   * Rotates the turret to the given angle relative to the robot.
   *
   * <p>The absolute encoder is configured so that one full rotation = 420°. The home position
   * (facing forward) is at encoder = {@link TurretConstants#kMaxRotationDeg} (210°).
   * Target angle of 0° = forward, +degrees = CCW, −degrees = CW.
   *
   * @param angleDegrees target angle in robot frame, clamped to ±kMaxRotationDeg
   */
  public void rotate(double angleDegrees) {
    angleDegrees = Math.max(-TurretConstants.kMaxRotationDeg,
                   Math.min( TurretConstants.kMaxRotationDeg, angleDegrees));

    // Map robot-relative angle to encoder position [0, 420°].
    double encoderSetpoint = angleDegrees + TurretConstants.kMaxRotationDeg;

    double currentPos = m_rotateEncoder.getPosition(); // degrees [0, 420]
    double error = encoderSetpoint - currentPos;

    if (Math.abs(error) > TurretConstants.kRotateToleranceDeg) {
      m_rotateMotor.set(TurretConstants.kRotatePower * Math.signum(error));
    } else {
      m_rotateMotor.set(0.0);
    }
  }

  /** Returns the current turret rotation angle relative to robot forward (degrees). */
  public double getRotationDeg() {
    return m_rotateEncoder.getPosition() - TurretConstants.kMaxRotationDeg;
  }

  /** Returns the current hood encoder position (rotations from min angle). */
  public double getHoodEncoderPos() {
    return m_hoodEncoder.getPosition();
  }

  /** Sets launch wheel power directly. Motor 2 runs opposite. For test mode only. */
  public void testRunLaunch(double power) {
    m_launchMotor1.set(power);
    m_launchMotor2.set(power);
  }

  /** Sets hood motor power directly. For test mode only. */
  public void testRunHood(double power) {
    m_hoodMotor.set(power);
  }

  /** Sets turret rotate motor power directly. For test mode only. */
  public void testRunRotate(double power) {
    m_rotateMotor.set(power);
  }

  public void setTestMode(boolean enabled) { m_testMode = enabled; }

  @Override
  public void periodic() {
    if (m_testMode) {
      m_testLaunch1PowerPub.set(m_launchMotor1.get());
      m_testLaunch1VelocityPub.set(m_launchMotor1.getEncoder().getVelocity());
      m_testLaunch2PowerPub.set(m_launchMotor2.get());
      m_testLaunch2VelocityPub.set(m_launchMotor2.getEncoder().getVelocity());
      m_testHoodPowerPub.set(m_hoodMotor.get());
      m_testHoodPositionPub.set(m_hoodEncoder.getPosition());
      m_testRotatePowerPub.set(m_rotateMotor.get());
      m_testRotatePositionPub.set(m_rotateEncoder.getPosition());
    }
  }
}
