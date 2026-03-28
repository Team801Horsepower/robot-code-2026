// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
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

  Pigeon2 RobotIMU = new Pigeon2(30);

  PIDController TurretRotatePID = new PIDController(1.5, 0, 0);
  SimpleMotorFeedforward TurretRotateFeedForward = new SimpleMotorFeedforward(0, 0);
  PIDController TurretHoodPID = new PIDController(0.9, 0.003, 0);
  SimpleMotorFeedforward TurretHoodFeedForward = new SimpleMotorFeedforward(0, 0, 0);
  PIDController ShooterPID = new PIDController(0.0021, 0.00085, 0.000085);
  SimpleMotorFeedforward ShooterFeedForward = new SimpleMotorFeedforward(0.0, 0.00195, 0.0);
  
  private final QuestSubsystem questNav;

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

  // Robot Position Variables
  public double TurretX;
  public double TurretY;
  public double TurretYaw;

  public double GoalX;
  public double GoalY;

  public double GoalOffsetX;
  public double GoalOffsetY;

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
    s_HoodTiltMotor.getEncoder().setPosition(0);

    ShooterPID.setIZone(50.0);
  }

  public void setTestMode(boolean enabled) { m_testMode = enabled; }
  public void setHoodAutoAim(boolean enabled) { m_hoodAutoAimEnabled = enabled; }
  public void testRunLaunch(double power) { s_ShooterMotorLeft.set(power); }
  public void testRunHood(double power) { s_HoodTiltMotor.set(power); }
  public void testRunRotate(double power) { s_TurretRotateMotor.set(power); }

  @Override
  public void periodic() {

    if (m_testMode) {
      return;
    }

    // Only run pose-dependent aiming if QuestNav has valid tracking data
    if (questNav.isTracking()) {
      /*
       * Takes robot pose2d published by QuestNav (Position of Robot)
       * and offsets it to the center of the turret.
       */
      TurretPose = questNav.RobotPose.transformBy(RobotToTurret);
      TurretX = TurretPose.getX();
      TurretY = TurretPose.getY();
      TurretRotation = TurretPose.getRotation();
      TurretYaw = TurretRotation.getZ();

      RobotIMU.ge
      RobotIMU.getAccelerationX();

      /*
       * Creates a 2D unit vector from the robot to the goal.
       * Calculates distance to goal.
       * Aims at 1 of 6 field coordinates depending on robot location (zone) and alliance color
       */
      if (m_alliance == Alliance.Red) {
        if (TurretX < 11.915394 && TurretY < 4.034536) {
          GoalX = Constants.TurretSubsystemConstants.AimPointR1.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointR1.getY();
        }
        else if (TurretX < 11.915394 && TurretY > 4.034536) {
          GoalX = Constants.TurretSubsystemConstants.AimPointR2.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointR2.getY();
        }
        else {
          GoalX = Constants.TurretSubsystemConstants.RedAllianceGoal.getX();
          GoalY = Constants.TurretSubsystemConstants.RedAllianceGoal.getY();
        }
      }
      else {
        if (TurretX > 4.625594 && TurretY < 4.034536) {
          GoalY = Constants.TurretSubsystemConstants.AimPointB2.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointB2.getY();
        }
        else if (TurretX > 4.625594 && TurretY > 4.034536) {
          GoalY = Constants.TurretSubsystemConstants.AimPointB1.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointB1.getY();
        }
        else {
          GoalX = Constants.TurretSubsystemConstants.BlueAllianceGoal.getX();
          GoalY = Constants.TurretSubsystemConstants.BlueAllianceGoal.getY();
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
      BallVelocityTarget = 6 - 0.00447 * DistanceToGoal + 0.104 * Math.pow(DistanceToGoal, 2);
      ShooterVelocityTarget = (60 * BallVelocityTarget) / (Constants.TurretSubsystemConstants.ShooterWheelCircumference);
    

      /*
       * TURRET ROTATE
       * Takes the tanget of the 2D unit vector to get the heading of the goal relative the robot.
       * Subtracts the position of the turret from the rotation of the robot to get the true angle of the turret.
       * Feedforward helps to overcome system resistance.
       * Feedback drives turret motor to target turret rotate theta.
       */
      double TurretThetaTargetRaw1 = -Math.atan2(vY, vX)
        + TurretYaw
        + Constants.TurretSubsystemConstants.TurretRotateScoreOffset;

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
      HoodThetaTarget = MathUtil.clamp(
        (0.0136 + 0.234 * DistanceToGoal + -0.0205 * Math.pow(DistanceToGoal, 2)),
        0.261799, 0.785398
      );

  } // end isTracking guard

  else {
    // Shooter
    ShooterVelocityTarget = 1754.463941;
  
    //Hood
    HoodThetaTarget = 0.785398;
    
    // Rotate
    TurretThetaTarget = 0.0;

  }

    /*
     * Turret Rotate PID
     * Turret Rotate Encoders
     */
    TurretThetaActual = s_TurretRotateEncoder.get() - Constants.TurretSubsystemConstants.TurretRotateOffset;
    s_TurretRotateMotor.set(
      (TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) +
      (TurretRotateFeedForward.calculate(0))
      );

    /*
     * Turret Shoter PID
     * Turret Shooter Encoders
     */
    s_ShooterMotorLeft.setVoltage(-1 * ShooterVelocityPIDSet);
    s_ShooterMotorRight.setVoltage(ShooterVelocityPIDSet);

    ShooterEncoder = s_ShooterMotorRight.getEncoder();
    ShooterVelocityActual = ShooterEncoder.getVelocity();

    ShooterVelocityPIDSet = ShooterPID.calculate(ShooterVelocityActual, ShooterVelocityTarget) + ShooterFeedForward.calculate(ShooterVelocityTarget);

    /*
     * Turret Hood Encoders
     */
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;

    if (m_hoodAutoAimEnabled) {
      HoodAim();
    } else {
      HoodReset();
    }
  }

  public void HoodAim() {
    s_HoodTiltMotor.set(
      (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
      (TurretHoodFeedForward.calculate(0))
    );
  }

  public void HoodReset() {
    double HoodResetTarget = 0.261799;
    s_HoodTiltMotor.set(
      (TurretHoodPID.calculate(HoodThetaActual, HoodResetTarget)) +
      (TurretHoodFeedForward.calculate(0))
    );
  }
}
