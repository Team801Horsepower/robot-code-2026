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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldZone;
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
  private final Drive drive;

  Transform3d RobotToTurret = new Transform3d(
    Constants.TurretSubsystemConstants.RobotToTurretX, 
    Constants.TurretSubsystemConstants.RobotToTurretY, 
    Constants.TurretSubsystemConstants.RobotToTurretZ, 
    new Rotation3d(
      Constants.TurretSubsystemConstants.RobotToTurretRoll, 
      Constants.TurretSubsystemConstants.RobotToTurretPitch, 
      Constants.TurretSubsystemConstants.RobotToTurretYaw));

  Transform3d TurretToForward = new Transform3d(-0.09, -0.09405, 0, new Rotation3d(0, 0, 0));
  Transform3d QuestToTurret = new Transform3d(-0.016,0.323,0, new Rotation3d(0,0,3.14159));

  public Pose3d TurretPose = new Pose3d();
  public Rotation2d RobotRotation3D = new Rotation2d();
  public Rotation3d QuestRotation = new Rotation3d();

  StructPublisher<Pose2d> turretPosePublisher = NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();

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
  private boolean m_turretAutoAimEnabled = true;
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

  public TurretSubsystem(QuestSubsystem questNav, Drive drive) {
    this.questNav = questNav;
    this.drive = drive;

    s_HoodTiltMotor.getEncoder().setPosition(0);

    ShooterPID.setIZone(50.0);
  }

  public void setTestMode(boolean enabled) { m_testMode = enabled; }
  public void setHoodAutoAim(boolean enabled) { m_hoodAutoAimEnabled = enabled; }
  public void setTurretAutoAim(boolean enabled) {m_turretAutoAimEnabled = enabled; }
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
    // TurretPose = questNav.QuestPose;
    TurretPose = questNav.QuestPose.transformBy(QuestToTurret);
    //RobotRotation = TurretPose.getRotation();
    RobotRotation3D = drive.getPose2d().getRotation();
    double RobotRotation = RobotRotation3D.getRadians();

    turretPosePublisher.set(TurretPose.toPose2d());

    double TurretToGoalX = vX;
    double TurretToGoalY = vY;

    // Turret Yaw
    TurretYaw = (-1 * (Math.atan2(TurretToGoalY, TurretToGoalX))) + RobotRotation;

    // Robot Velocity  * * * * *
    TurretVx = VelocityFilterX.calculate((questNav.RobotPose.getX() - TurretXMinusOne)/ 0.02);
    TurretVy = VelocityFilterY.calculate((questNav.RobotPose.getY() - TurretYMinusOne) / 0.02);

    // Time of Flight (Dynamic Distance to Goal) * * * * *
    double staticGoalX = GoalX - TurretPose.getX();
    double staticGoalY = GoalY - TurretPose.getY();
    double staticDistanceToGoal = Math.sqrt((staticGoalX * staticGoalX) + (staticGoalY * staticGoalY));
    TimeOfFlight = 0.976 + 0.0027 * staticDistanceToGoal + 0.00677 * (staticDistanceToGoal * staticDistanceToGoal);

    // Distance to Goal (Dynamic)
    DistanceToGoal = Math.sqrt((vX * vX) + (vY * vY));
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
    if (questNav.isTracking() && m_turretAutoAimEnabled) {

      // Target Position  * * * * *
        GoalX = Constants.TurretSubsystemConstants.AimPointR1.getX();
        GoalY = Constants.TurretSubsystemConstants.AimPointR1.getY();

      //Goal Position
      vX = GoalX - TurretPose.getX();
      vY = GoalY - TurretPose.getY();

    
      // Calculate Turret Rotate * * * * *
      double TurretThetaTargetRaw1 = TurretYaw;
      
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
      HoodThetaTarget = (0.508);

      // Calculate Turret Shooter  * * * * *
      ShooterVelocityTarget = 1700;
    }
/*
 * o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o     o
 */

/*
 * IF QUEST NAV IS NOT TRACKING ------------------------------------------------------------------------------------------------
 */
    else {
      // Shooter  * * * * *
      ShooterVelocityTarget = 1700;
    
      //Hood  * * * * *
      HoodThetaTarget = 0.508;
      
      // Rotate  * * * * *
      TurretThetaTarget = -1.5708;

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
 * SMART DASHBOARD -------------------------------------------------------------------------------------------------------------
 */
    // NA
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
