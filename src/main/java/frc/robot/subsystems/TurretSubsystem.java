// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

  SparkFlex s_ShooterMotorLeft = new SparkFlex(Constants.TurretSubsystemConstants.FlywheelMotorLeftCANID, MotorType.kBrushless);
  SparkFlex s_ShooterMotorRight = new SparkFlex(Constants.TurretSubsystemConstants.FlywheelMotorRightCANID, MotorType.kBrushless);
  SparkFlex s_TurretRotateMotor = new SparkFlex(Constants.TurretSubsystemConstants.TurretRotateMotorCANID, MotorType.kBrushless);
  SparkFlex s_HoodTiltMotor = new SparkFlex(Constants.TurretSubsystemConstants.HoodTiltMotorCANID, MotorType.kBrushless);

  DutyCycleEncoder s_TurretRotateEncoder = new DutyCycleEncoder(Constants.TurretSubsystemConstants.TurretRotateEncoderDIOID, Constants.TurretSubsystemConstants.TurretRotateFreedom, 0);
  private RelativeEncoder HoodEncoder;
  private RelativeEncoder ShooterEncoder;

  PIDController TurretRotatePID = new PIDController(1.5, 0, 0);
  SimpleMotorFeedforward TurretRotateFeedForward = new SimpleMotorFeedforward(0, 0);
  PIDController TurretHoodPID = new PIDController(0.9, 0.003, 0);
  SimpleMotorFeedforward TurretHoodFeedForward = new SimpleMotorFeedforward(0, 0, 0);
  PIDController ShooterPID = new PIDController(0.0021, 0.00085, 0.000085);
  SimpleMotorFeedforward ShooterFeedForward = new SimpleMotorFeedforward(0.0, 0.00195, 0.0);
  
  private final QuestSubsystem questNav;
  private final Supplier<ChassisSpeeds> m_chassisSpeedsSupplier;

  Transform3d RobotToTurret = new Transform3d(
    Constants.TurretSubsystemConstants.RobotToTurretX, 
    Constants.TurretSubsystemConstants.RobotToTurretY, 
    Constants.TurretSubsystemConstants.RobotToTurretZ, 
    new Rotation3d(
      Constants.TurretSubsystemConstants.RobotToTurretRoll, 
      Constants.TurretSubsystemConstants.RobotToTurretPitch, 
      Constants.TurretSubsystemConstants.RobotToTurretYaw));

  public Pose3d TurretPose = new Pose3d();
  public Rotation3d TurretRotation = new Rotation3d();

  // Vector Variables
  public double vX;
  public double vY;
  public double vZ;

  // Robot Position Variables
  public double TurretX;
  public double TurretY;
  public double TurretZ;
  public double TurretYaw;

  //Turret Variables
  public double TicksPerDegree;
  public double TurretThetaActual;
  public double TurretThetaTarget;

  // Hood Variables
  public double HoodThetaActual;
  public double HoodThetaTarget;
  public double DistanceToGoal;

  // Shooter Variables
  public double BallVelocityTarget;
  public double ShooterVelocityTarget;
  public double ShooterVelocityActual;
  public double ShooterVelocityPIDSet;

  private boolean m_testMode = false;
  private boolean m_hoodAutoAimEnabled = false;

  //Alliance
  public Optional<Alliance> AllianceColor;
  public double GoalX;
  public double GoalY;

  private edu.wpi.first.wpilibj.DriverStation.Alliance m_alliance =
      edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

  /**
   * Caches the current alliance color. Call from autonomousInit/teleopInit.
   * Defaults to Blue if not set.
   */
  public void setAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
    m_alliance = alliance;
  }

  public TurretSubsystem(QuestSubsystem questNav, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.questNav = questNav;
    this.m_chassisSpeedsSupplier = chassisSpeedsSupplier;
    s_HoodTiltMotor.getEncoder().setPosition(0);

    ShooterPID.setIZone(50.0);

    //SmartDashboard.putNumber("BallVelocityTarget", 0);

    //SmartDashboard.putNumber("Shooter P", 0.0023);
    //SmartDashboard.putNumber("Shooter I", 0.0005);
    //SmartDashboard.putNumber("Shooter D", 0.0005);
    //SmartDashboard.putNumber("Shooter Ks", 0.0);
    //SmartDashboard.putNumber("Shooter Kv", 0.00195);
    //SmartDashboard.putNumber("Shooter Ka", 0.0);

    //SmartDashboard.putNumber("ShotMultiplier", 0.95);
  }

  public void setTestMode(boolean enabled) { m_testMode = enabled; }
  public void setHoodAutoAim(boolean enabled) { m_hoodAutoAimEnabled = enabled; }
  public void testRunLaunch(double power) { s_ShooterMotorLeft.set(power); }
  public void testRunHood(double power) { s_HoodTiltMotor.set(power); }
  public void testRunRotate(double power) { s_TurretRotateMotor.set(power); }

  @Override
  public void periodic() {
    //double newP = SmartDashboard.getNumber("Shooter P", 0.0023);
    //double newI = SmartDashboard.getNumber("Shooter I", 0.0005);
    //double newD = SmartDashboard.getNumber("Shooter D", 0.0005);
    //ShooterPID.setP(newP);
    //ShooterPID.setI(newI);
    //ShooterPID.setD(newD);
    //double newKs = SmartDashboard.getNumber("Shooter Ks", 0.0);
    //double newKv = SmartDashboard.getNumber("Shooter Kv", 0.00195);
    //double newKa = SmartDashboard.getNumber("Shooter Ka", 0.0);
    //ShooterFeedForward.setKs(newKs);
    //ShooterFeedForward.setKv(newKv);
    //ShooterFeedForward.setKa(newKa);

    if (m_testMode) {
      return;
    }

    // Only run pose-dependent aiming if QuestNav has valid tracking data and is not at (0,0)
    if (questNav.isTracking()) {
      // && questNav.RobotPose.getX() > 0.1 && questNav.RobotPose.getY() > 0.1
      /*
       * Takes robot pose2d published by QuestNav (Position of Quest, NOT position of center of robot)
       * and offsets it to the center of the turret.
       */
      TurretPose = questNav.RobotPose.transformBy(RobotToTurret);
      TurretX = TurretPose.getX();
      TurretY = TurretPose.getY();
      TurretZ = TurretPose.getZ();
      TurretRotation = TurretPose.getRotation();
      TurretYaw = TurretRotation.getZ();

      /*
       * Creates a 2D unit vector from the robot to the goal.
       * Calculates distance to goal.
       * Aims at 1 of 6 field coordinates depending on robot location (zone) and alliance color
       */
      if (m_alliance == Alliance.Red) {
        if (TurretX < 11.915394 && TurretY < 4.034536) {
          vX = Constants.TurretSubsystemConstants.AimPointR1.getX() - TurretX;
          vY = Constants.TurretSubsystemConstants.AimPointR1.getY() - TurretY;
          vZ = Constants.TurretSubsystemConstants.AimPointR1.getZ() - TurretZ;
        }
        else if (TurretX < 11.915394 && TurretY > 4.034536) {
          vX = Constants.TurretSubsystemConstants.AimPointR2.getX() - TurretX;
          vY = Constants.TurretSubsystemConstants.AimPointR2.getY() - TurretY;
          vZ = Constants.TurretSubsystemConstants.AimPointR2.getZ() - TurretZ;
        }
        else {
          vX = Constants.TurretSubsystemConstants.RedAllianceGoal.getX() - TurretX;
          vY = Constants.TurretSubsystemConstants.RedAllianceGoal.getY() - TurretY;
          vZ = Constants.TurretSubsystemConstants.RedAllianceGoal.getZ() - TurretZ;
        }
      }
      else {
        if (TurretX > 4.625594 && TurretY < 4.034536) {
          vX = Constants.TurretSubsystemConstants.AimPointB2.getX() - TurretX;
          vY = Constants.TurretSubsystemConstants.AimPointB2.getY() - TurretY;
          vZ = Constants.TurretSubsystemConstants.AimPointB2.getZ() - TurretZ;
        }
        else if (TurretX > 4.625594 && TurretY > 4.034536) {
          vX = Constants.TurretSubsystemConstants.AimPointB1.getX() - TurretX;
          vY = Constants.TurretSubsystemConstants.AimPointB1.getY() - TurretY;
          vZ = Constants.TurretSubsystemConstants.AimPointB1.getZ() - TurretZ;
        }
        else {
          vX = Constants.TurretSubsystemConstants.BlueAllianceGoal.getX() - TurretX;
          vY = Constants.TurretSubsystemConstants.BlueAllianceGoal.getY() - TurretY;
          vZ = Constants.TurretSubsystemConstants.BlueAllianceGoal.getZ() - TurretZ;
        }
      }
    
      DistanceToGoal = Math.sqrt(vX*vX + vY*vY);
      SmartDashboard.putNumber("DistanceToGoal", DistanceToGoal);

      //double ShotMultiplier = SmartDashboard.getNumber("ShotMultiplier", 0.95);

      /*
      * TURRET SHOOTER
      * Calculates the shooter velocity target based on the line of best fit fron an analysis of physics equations.
      * Converts the ball velocity in m/s to motor speed in RPM.
      * Gets the velocity reading from the left flywheel motor.
      * Feedforward drives flywheel to target velocity.
      * Feedback drives flywheel to target velocity. 
      */
      // Ball velocity needed for both lead compensation and shooter control
      // BallVelocityTarget = 5.58 + 0.38 * DistanceToGoal + 0.0394 * Math.pow(DistanceToGoal, 2);
      //BallVelocityTarget = SmartDashboard.getNumber("BallVelocityTarget", 0);
      BallVelocityTarget = 6 - 0.00447 * DistanceToGoal + 0.104 * Math.pow(DistanceToGoal, 2);

      ShooterVelocityTarget = (60 * BallVelocityTarget) / (Constants.TurretSubsystemConstants.ShooterWheelCircumference);
      //ShooterEncoder = s_ShooterMotorRight.getEncoder();
      //ShooterVelocityActual = ShooterEncoder.getVelocity();

      /*
       * TURRET ROTATE
       * Takes the tanget of the 2D unit vector to get the heading of the goal relative the robot.
       * Subtracts the position of the turret from the rotation of the robot to get the true angle of the turret.
       * Feedforward helps to overcome system resistance.
       * Feedback drives turret motor to target turret rotate theta.
       */
      TurretThetaActual = s_TurretRotateEncoder.get() - Constants.TurretSubsystemConstants.TurretRotateOffset;

      double TurretThetaTargetRaw1 = -Math.atan2(vY, vX)
        + TurretYaw
        + Constants.TurretSubsystemConstants.TurretRotateScoreOffset;

      // ── Lead + radial velocity compensation ─────────────────────────────
      // Skip velocity calc if too close to goal (avoid division by zero)
      double leadOffset = 0.0;
      double vRadial = 0.0;
      double adjustedBallVelocity = BallVelocityTarget;
      double effectiveDistance = DistanceToGoal;
      if (DistanceToGoal > 0.1 && BallVelocityTarget > 0.1) {
        // Convert robot-relative chassis speeds to field-relative
        // Note: TurretYaw equals robot heading because RobotToTurretYaw = 0.0
        // Note: ChassisSpeeds source is CTRE odometry; heading is from QuestNav
        ChassisSpeeds robotSpeeds = m_chassisSpeedsSupplier.get();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, Rotation2d.fromRadians(TurretYaw));

        // Unit vector from turret to goal
        double uX = vX / DistanceToGoal;
        double uY = vY / DistanceToGoal;

        // Perpendicular velocity (90° CCW rotation of unit vector)
        double vPerp = fieldSpeeds.vxMetersPerSecond * (-uY)
                     + fieldSpeeds.vyMetersPerSecond * uX;

        // Radial velocity: dot product of field velocity with unit vector toward goal
        // Positive = approaching goal, negative = retreating
        vRadial = fieldSpeeds.vxMetersPerSecond * uX
                + fieldSpeeds.vyMetersPerSecond * uY;

        // Disable radial compensation if robot speed is too large relative to ball speed
        boolean radialSafe = Math.abs(vRadial) < 0.8 * BallVelocityTarget;

        double effectiveBallSpeed = BallVelocityTarget;
        if (radialSafe) {
            effectiveBallSpeed += vRadial * Constants.TurretSubsystemConstants.kRadialVelocityFactor;
        }
        effectiveBallSpeed = Math.max(effectiveBallSpeed, 1.0);

        // Lead offset using corrected flight time
        leadOffset = Math.atan(vPerp / effectiveBallSpeed)
                          * Constants.TurretSubsystemConstants.kLeadFactor;

        if (radialSafe) {
          // Subtract radial velocity from muzzle velocity — ball inherits robot's radial motion
          adjustedBallVelocity = BallVelocityTarget
              - vRadial * Constants.TurretSubsystemConstants.kRadialVelocityFactor;
          adjustedBallVelocity = Math.max(adjustedBallVelocity, 1.0);
          ShooterVelocityTarget = (60 * adjustedBallVelocity)
              / Constants.TurretSubsystemConstants.ShooterWheelCircumference;

          // Effective distance: predict actual ball travel distance accounting for radial motion
          double flightTime = DistanceToGoal / effectiveBallSpeed;
          effectiveDistance = DistanceToGoal
              - vRadial * Constants.TurretSubsystemConstants.kRadialVelocityFactor * flightTime;
          effectiveDistance = Math.max(effectiveDistance, 0.5);
        }
      }
      SmartDashboard.putNumber("LeadOffset", Math.toDegrees(leadOffset));
      SmartDashboard.putNumber("RadialVelocity", vRadial);
      SmartDashboard.putNumber("AdjustedBallVelocity", adjustedBallVelocity);
      SmartDashboard.putNumber("EffectiveDistance", effectiveDistance);

      TurretThetaTargetRaw1 += leadOffset;

      while (TurretThetaTargetRaw1 < -Math.PI) {
        TurretThetaTargetRaw1 += 2.0 * Math.PI;
      }
      while (TurretThetaTargetRaw1 > Math.PI) {
        TurretThetaTargetRaw1 -= 2.0 * Math.PI;
      }
      double TurretThetaTargetRaw2 = TurretThetaTargetRaw1;
      double BestDist = 2.0 * Math.PI;
      for (int i = -1; i <= 1; i++) {
        double PossibleTarget = TurretThetaTargetRaw1 + (double)i * 2.0 * Math.PI;
        double Min = -3.49066;
        double Max = 3.49066;
        if (PossibleTarget <= Min || PossibleTarget >= Max) {
          continue;
        }
        double Dist = Math.abs(PossibleTarget - TurretThetaActual);
        if (Dist >= BestDist) {
          continue;
        }
        BestDist = Dist;
        TurretThetaTargetRaw2 = PossibleTarget;
      }
      TurretThetaTarget = MathUtil.clamp(
        TurretThetaTargetRaw2,
        -3.49066,
        3.49066
      );
    
      /*
       * TURRET HOOD — zone-aware
       * Launch zone: auto-aim hood based on distance-to-goal polynomial.
       * Trench zone: retract hood to clear the trench (drive to minimum angle).
       * Far zone: skip hood control entirely (future: alternate aiming).
       */
      HoodEncoder = s_HoodTiltMotor.getEncoder();
      HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
      // Flat-ground polynomial baseline
      double hoodPolynomial = 0.0136 + 0.234 * effectiveDistance + -0.0205 * Math.pow(effectiveDistance, 2);

      // Elevation correction for actual height difference vs flat-ground baseline
      double elevationCorrection = Math.atan2(vZ, DistanceToGoal)
          - Math.atan2(Constants.TurretSubsystemConstants.BaselineDeltaZ, DistanceToGoal);

      // Desired field-frame launch elevation
      double desiredFieldElevation = hoodPolynomial + elevationCorrection;

      // Construct desired field-frame launch direction unit vector
      double aimYawField = Math.atan2(vY, vX);
      double cElev = Math.cos(desiredFieldElevation);
      double sElev = Math.sin(desiredFieldElevation);
      Translation3d fieldDir = new Translation3d(
          cElev * Math.cos(aimYawField),
          cElev * Math.sin(aimYawField),
          sElev);

      // Inverse-rotate from field frame to robot frame.
      // Handles all pitch/roll/yaw coupling — turret yaw no longer
      // controls pure horizontal aim when the robot is tilted.
      Translation3d robotDir = fieldDir.rotateBy(TurretRotation.unaryMinus());

      // Extract required robot-frame hood angle
      double robotDirHoriz = Math.sqrt(
          robotDir.getX() * robotDir.getX() + robotDir.getY() * robotDir.getY());
      double requiredHoodAngle = Math.atan2(robotDir.getZ(), robotDirHoriz);

      HoodThetaTarget = MathUtil.clamp(requiredHoodAngle, 0.261799, 0.785398);

      SmartDashboard.putNumber("Hood/Polynomial", Math.toDegrees(hoodPolynomial));
      SmartDashboard.putNumber("Hood/ElevationCorrection", Math.toDegrees(elevationCorrection));
      SmartDashboard.putNumber("Hood/RequiredHoodAngle", Math.toDegrees(requiredHoodAngle));
      SmartDashboard.putNumber("Hood/TurretZ", TurretZ);
      SmartDashboard.putNumber("Hood/RobotPitch", Math.toDegrees(TurretRotation.getY()));
      SmartDashboard.putNumber("Hood/RobotRoll", Math.toDegrees(TurretRotation.getX()));

  } // end isTracking guard

  else {
    // Shooter
    ShooterVelocityTarget = 1754.463941;
  
    //Hood
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
    HoodThetaTarget = 0.785398;
    
    // Rotate
    TurretThetaActual = s_TurretRotateEncoder.get() - Constants.TurretSubsystemConstants.TurretRotateOffset;
    TurretThetaTarget = 0.0;

  }

    /*
     * Turret Rotate PID
     */
    s_TurretRotateMotor.set(
      (TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) +
      (TurretRotateFeedForward.calculate(0))
      );

    SmartDashboard.putNumber("TurretEncoderActual", TurretThetaActual);
    SmartDashboard.putNumber("TurretPositionTarget", TurretThetaTarget);
    SmartDashboard.putData("TurretRotatePID", TurretRotatePID);

    /*
     * Turret Shoter PID
     */
    ShooterEncoder = s_ShooterMotorRight.getEncoder();
    ShooterVelocityActual = ShooterEncoder.getVelocity();

    ShooterVelocityPIDSet = ShooterPID.calculate(ShooterVelocityActual, ShooterVelocityTarget)
        + ShooterFeedForward.calculate(ShooterVelocityTarget);

    s_ShooterMotorLeft.setVoltage(-1 * ShooterVelocityPIDSet);
    s_ShooterMotorRight.setVoltage(ShooterVelocityPIDSet);

    SmartDashboard.putNumber("ShooterVelocityTarget", ShooterVelocityTarget);
    SmartDashboard.putNumber("ShooterVelocityActual", ShooterVelocityActual);
    SmartDashboard.putData("ShooterPID", ShooterPID);

    if (m_hoodAutoAimEnabled) {
      HoodAim();
    } else {
      HoodReset();
    }
  }

  public void HoodAim() {
    double hoodOutput = TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)
        + TurretHoodFeedForward.calculate(0);
    s_HoodTiltMotor.set(hoodOutput);
    SmartDashboard.putNumber("Hood/ActualAngle", Math.toDegrees(HoodThetaActual));
    SmartDashboard.putNumber("Hood/MotorPower", hoodOutput);
  }

  public void HoodReset() {
    double HoodResetTarget = 0.261799;
    double hoodOutput = TurretHoodPID.calculate(HoodThetaActual, HoodResetTarget)
        + TurretHoodFeedForward.calculate(0);
    s_HoodTiltMotor.set(hoodOutput);
    SmartDashboard.putNumber("Hood/ActualAngle", Math.toDegrees(HoodThetaActual));
    SmartDashboard.putNumber("Hood/MotorPower", hoodOutput);
  }
}
