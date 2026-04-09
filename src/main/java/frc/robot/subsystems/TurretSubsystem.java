// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

import java.util.Optional;

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
  PIDController TurretHoodPID = new PIDController(0.7, 0, 0.0085);
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

  Transform3d TurretToForward = new Transform3d(-0.2, -0.09405, 0, new Rotation3d(0, 0, 0));

  public Pose3d TurretPose = new Pose3d();
  public Pose3d TurretForwardPose = new Pose3d();
  public Rotation3d RobotRotation = new Rotation3d();

  StructPublisher<Pose2d> turretPosePublisher = NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> turretForwardPosePublisher = NetworkTableInstance.getDefault().getStructTopic("TurretForwardPose", Pose2d.struct).publish();

  LinearFilter VelocityFilterX = LinearFilter.movingAverage(5);
  LinearFilter VelocityFilterY = LinearFilter.movingAverage(5);

/*
 * TURRET SUBSYSTEM VARIABLES ----------------------------------------------------------------------------------------------------
 */
  // Vector Variables  * * * * *
  public double vX;
  public double vY;

  public double GoalX;
  public double GoalY;

  public double GoalOffsetX;
  public double GoalOffsetY;

  // Robot Position Variables  * * * * *
  public double TurretYaw;

  public double TurretXMinusOne;
  public double TurretYMinusOne;

  // Robot Velocity Vectors  * * * * *
  public double TurretVx;
  public double TurretVy;

  public double TimeOfFlight;

  //Turret Variables  * * * * *
  public double TicksPerDegree;
  public double TurretThetaActual;
  public double TurretThetaTarget;
  public double TurretThetaTargetRaw1;
  public double TurretThetaTargetRaw2;

  // Hood Variables  * * * * *
  public double HoodThetaActual;
  public double HoodThetaTarget;
  public double DistanceToGoal;

  // Shooter Variables  * * * * *
  public double BallVelocityTarget;
  public double ShooterVelocityTarget;
  public double ShooterVelocityActual;
  public double ShooterVelocityPIDSet;

  private boolean m_testMode = false;
  private boolean m_hoodAutoAimEnabled = false;
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * GET ALLIANCE COLOR ------------------------------------------------------------------------------------------------------------
 */
  public Optional<Alliance> AllianceColor;

  private edu.wpi.first.wpilibj.DriverStation.Alliance m_alliance =
      edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

  public void setAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
    m_alliance = alliance;
  }
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

  public TurretSubsystem(QuestSubsystem questNav) {
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

/*
 * GET ENCODER VALUES ----------------------------------------------------------------------------------------------------------
 */
    // Turret Rotate Encoder  * * * * *
    TurretThetaActual = s_TurretRotateEncoder.get() - Constants.TurretSubsystemConstants.TurretRotateOffset;

    // Turret Hood Encoder  * * * * *
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;

    // Turret Shooter Encoder  * * * * *
    ShooterEncoder = s_ShooterMotorRight.getEncoder();
    ShooterVelocityActual = ShooterEncoder.getVelocity();
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * GET ROBOT POSITION AND VELOCITY ---------------------------------------------------------------------------------------------
 */
    // Robot Position  * * * * *
    TurretPose = questNav.RobotPose.transformBy(RobotToTurret);
    TurretForwardPose = questNav.RobotPose.transformBy(TurretToForward);

    RobotRotation = TurretPose.getRotation();

    turretPosePublisher.set(TurretPose.toPose2d());
    turretForwardPosePublisher.set(TurretForwardPose.toPose2d());

    double TurretToGoalX = vX; //a1
    double TurretToGoalY = vY; //a2

    double TurretForwardX = TurretForwardPose.getX() - TurretPose.getX(); //b1
    double TurretForwardY = TurretForwardPose.getY() - TurretPose.getY(); //b2

    // Turret Yaw
    TurretYaw = Math.atan2(((TurretToGoalX * TurretForwardY) - (TurretToGoalY * TurretForwardX)), ((TurretToGoalX * TurretForwardX) + (TurretForwardY * TurretToGoalY))) + 3.14159;

    // Robot Velocity  * * * * *
    TurretVx = VelocityFilterX.calculate((TurretPose.getX() - TurretXMinusOne) / 0.02);
    TurretVy = VelocityFilterY.calculate((TurretPose.getY() - TurretYMinusOne) / 0.02);

    // Distance to Goal
    DistanceToGoal = Math.sqrt((TurretToGoalX * TurretToGoalX) + (TurretToGoalY * TurretToGoalY));
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

    if (m_testMode) {
      return;
    }

/*
 * IF QUEST NAV IS TRACKNG -----------------------------------------------------------------------------------------------------
 */
    if (questNav.isTracking()) {

      // Target Position  * * * * *
      if (m_alliance == Alliance.Red) {
        if (TurretPose.getX() < 11.915394 && TurretPose.getY() < 4.034536) {
          GoalX = Constants.TurretSubsystemConstants.AimPointR1.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointR1.getY();
        }
        else if (TurretPose.getX() < 11.915394 && TurretPose.getY() > 4.034536) {
          GoalX = Constants.TurretSubsystemConstants.AimPointR2.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointR2.getY();
        }
        else {
          GoalX = Constants.TurretSubsystemConstants.RedAllianceGoal.getX();
          GoalY = Constants.TurretSubsystemConstants.RedAllianceGoal.getY();
        }
      }
      else {
        if (TurretPose.getX() > 4.625594 && TurretPose.getY() < 4.034536) {
          GoalX = Constants.TurretSubsystemConstants.AimPointB2.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointB2.getY();
        }
        else if (TurretPose.getX() > 4.625594 && TurretPose.getY() > 4.034536) {
          GoalX = Constants.TurretSubsystemConstants.AimPointB1.getX();
          GoalY = Constants.TurretSubsystemConstants.AimPointB1.getY();
        }
        else {
          GoalX = Constants.TurretSubsystemConstants.BlueAllianceGoal.getX();
          GoalY = Constants.TurretSubsystemConstants.BlueAllianceGoal.getY();
        }
      }

      // Adjust Goal Position For Robot Velocity * * * * *
      vX = (GoalX - (TurretVx * TimeOfFlight)) - TurretPose.getX();
      vY = (GoalY - (TurretVy * TimeOfFlight)) - TurretPose.getY();

      // Calculate Turret Rotate * * * * *
      double TurretThetaTargetRaw1 =
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

      // Calculate  Turret Hood  * * * * *
      HoodEncoder = s_HoodTiltMotor.getEncoder();
      HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
      HoodThetaTarget = MathUtil.clamp(
        (0.0136 + 0.234 * DistanceToGoal + -0.0205 * Math.pow(DistanceToGoal, 2)),
        0.261799, 0.785398
      );

      // Calculate Turret Shooter  * * * * *
      BallVelocityTarget = 6 - 0.00447 * DistanceToGoal + 0.104 * Math.pow(DistanceToGoal, 2);
      ShooterVelocityTarget = (60 * BallVelocityTarget) / (Constants.TurretSubsystemConstants.ShooterWheelCircumference);

  }

/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * IF QUEST NAV IS NOT TRACKING ------------------------------------------------------------------------------------------------
 */
    else {
      // Shooter  * * * * *
      ShooterVelocityTarget = 1754.463941;
    
      //Hood  * * * * *
      HoodThetaTarget = 0.785398;
      
      // Rotate  * * * * *
      TurretThetaTarget = -0.797;

    }
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * PID CONTROLLERS -------------------------------------------------------------------------------------------------------------
 */
    // Turret Rotate PID  * * * * *
    s_TurretRotateMotor.set(
      (TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) +
      (TurretRotateFeedForward.calculate(0))
      );

    // Turret Shooter PID  * * * * *
    ShooterVelocityPIDSet = ShooterPID.calculate(ShooterVelocityActual, ShooterVelocityTarget) + ShooterFeedForward.calculate(ShooterVelocityTarget);
    s_ShooterMotorLeft.setVoltage(-1 * ShooterVelocityPIDSet);
    s_ShooterMotorRight.setVoltage(ShooterVelocityPIDSet);
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * STORE PREVIOUS ROBOT POSITION -----------------------------------------------------------------------------------------------
 */
    TurretXMinusOne = TurretPose.getX();
    TurretYMinusOne = TurretPose.getY();
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * SMART DASHBOARD -------------------------------------------------------------------------------------------------------------
 */
    // Robot Velocity  * * * * *
    SmartDashboard.putNumber("TurretVx", TurretVx);
    SmartDashboard.putNumber("TurretVy", TurretVy);

    // Targeting  * * * * *
    SmartDashboard.putNumber("DistanceToGoal", DistanceToGoal);

    // Turret
    SmartDashboard.putData("ShooterPID", ShooterPID);
    SmartDashboard.putData("TurretRotatePID", TurretRotatePID);
    SmartDashboard.putNumber("TurretRotation", TurretYaw);
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

    if (m_hoodAutoAimEnabled) {
      HoodAim();
    } else {
      HoodReset();
    }
  }

/*
 * TURRET HOOD SET / RESET METHODS -----------------------------------------------------------------------------------------------
 */
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
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */
}
