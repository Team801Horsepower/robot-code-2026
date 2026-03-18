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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;

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
  PIDController ShooterPID = new PIDController(0.005, 0.001, 0);
  SimpleMotorFeedforward ShooterFeedForward = new SimpleMotorFeedforward(0, 0.0022, 0.0);
  
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

  private boolean m_testMode = false;
  private boolean m_hoodAutoAimEnabled = false;

  private edu.wpi.first.wpilibj.DriverStation.Alliance m_alliance =
      edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

  /**
   * Caches the current alliance color. Call from autonomousInit/teleopInit.
   * Defaults to Blue if not set.
   */
  public void setAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
    m_alliance = alliance;
  }

  public TurretSubsystem(QuestSubsystem questNav) {
    this.questNav = questNav;
    s_HoodTiltMotor.getEncoder().setPosition(0);

    SparkFlexConfig GlobalConfig = new SparkFlexConfig();
    SparkFlexConfig ShooterRightFollowerConfig = new SparkFlexConfig();
    ShooterRightFollowerConfig.inverted(true).apply((GlobalConfig).follow(s_ShooterMotorRight));

    SmartDashboard.putNumber("ShooterVelocityMultiplier", Constants.TurretSubsystemConstants.ShooterVelocityMultiplier);
    SmartDashboard.putNumber("ShooterVelocityEficiencey", Constants.TurretSubsystemConstants.ShooterVelcoityEfficiency);
    SmartDashboard.putNumber("TurretRotateScoreOffset", Constants.TurretSubsystemConstants.TurretRotateScoreOffset);
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
     * Takes robot pose2d published by QuestNav (Position of Quest, NOT position of center of robot)
     * and offsets it to the center of the turret.
     */
    TurretPose = questNav.RobotPose.transformBy(RobotToTurret);
    TurretX = TurretPose.getX();
    TurretY = TurretPose.getY();
    TurretRotation = TurretPose.getRotation();
    TurretYaw = TurretRotation.getZ();

    /*
     * Creates a 2D unit vector from the robot to the goal.
     * Calculates distance to goal.
     */
    double goalX, goalY;
    if (m_alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
      goalX = Constants.TurretSubsystemConstants.RedGoalX;
      goalY = Constants.TurretSubsystemConstants.RedGoalY;
    } else {
      goalX = Constants.TurretSubsystemConstants.BlueGoalX;
      goalY = Constants.TurretSubsystemConstants.BlueGoalY;
    }
    vX = goalX - TurretX;
    vY = goalY - TurretY;
    
    DistanceToGoal = Math.sqrt(vX*vX + vY*vY);

    /*
     * TURRET ROTATE
     * Takes the tanget of the 2D unit vector to get the heading of the goal relative the robot.
     * Subtracts the position of the turret from the rotation of the robot to get the true angle of the turret.
     * Feedforward helps to overcome system resistance.
     * Feedback drives turret motor to target turret rotate theta.
     */
    TurretThetaActual = s_TurretRotateEncoder.get() - Constants.TurretSubsystemConstants.TurretRotateOffset;
    // Theta target without accounting for 360ing
    double TurretThetaTargetRaw1 = -Math.atan2(vY, vX)
      + TurretYaw
      + Constants.TurretSubsystemConstants.TurretRotateScoreOffset;
    while (TurretThetaTargetRaw < -Math.PI) {
      TurretThetaTargetRaw += 2.0 * Math.PI;
    }
    while (TurretThetaTargetRaw > Math.PI) {
      TurretThetaTargetRaw -= 2.0 * Math.PI;
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
      3.49066,
    );
    // TurretThetaTarget = MathUtil.clamp(
    //   -Math.atan2(vY, vX)
    //     + TurretYaw
    //     + Constants.TurretSubsystemConstants.TurretRotateScoreOffset,
    //   -3.49066,
    //   3.49066,
    // );

    s_TurretRotateMotor.set(
    (TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) +
    (TurretRotateFeedForward.calculate(0))
    );
    
    /*
     * TURRET HOOD — zone-aware
     * Launch zone: auto-aim hood based on distance-to-goal polynomial.
     * Trench zone: retract hood to clear the trench (drive to minimum angle).
     * Far zone: skip hood control entirely (future: alternate aiming).
     */
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;

    if (m_hoodAutoAimEnabled) {
    Constants.FieldZone zone = Constants.FieldConstants.getFieldZone(
        questNav.RobotPose.getX(), m_alliance);

    switch (zone) {
      case LAUNCH:
        // Normal auto-aim: polynomial fit from physics analysis
        HoodThetaTarget = MathUtil.clamp(
          (0.0136 + 0.234 * DistanceToGoal + -0.0205 * Math.pow(DistanceToGoal, 2)),
          0.261799, 0.785398
        );
        s_HoodTiltMotor.set(
          (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
          (TurretHoodFeedForward.calculate(0))
        );
        break;

      case TRENCH:
        // Retract hood to minimum angle to clear the trench
        HoodThetaTarget = 0.261799;
        s_HoodTiltMotor.set(
          (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
          (TurretHoodFeedForward.calculate(0))
        );
        break;

      case FAR:
        // No hood control — future: alternate aiming behavior
        break;
    }
    } else {
      s_HoodTiltMotor.set(0);
    }
    } // end isTracking guard

    /*
     * TURRET SHOOTER
     * Calculates the shooter velocity target based on the line of best fit fron an analysis of physics equations.
     * Converts the ball velocity in m/s to motor speed in RPM.
     * Gets the velocity reading from the left flywheel motor.
     * Feedforward drives flywheel to target velocity.
     * Feedback drives flywheel to target velocity. 
     */
    BallVelocityTarget = 5.58 + 0.38 * DistanceToGoal + -0.0394 * Math.pow(DistanceToGoal, 2);
    ShooterVelocityTarget = (60 * BallVelocityTarget) / (Constants.TurretSubsystemConstants.ShooterWheelCircumference);
    ShooterEncoder = s_ShooterMotorLeft.getEncoder();
    ShooterVelocityActual = ShooterEncoder.getVelocity();

    s_ShooterMotorLeft.setVoltage(
      (ShooterPID.calculate(ShooterVelocityActual, ShooterVelocityTarget)) +
      (ShooterFeedForward.calculate(ShooterVelocityTarget))
    );

    /*
     * Publishes several values to Smart Dashboard which can be accessed in advantagescope under the Smart Dashboard topic.
     */
    // Turret Numbers
    SmartDashboard.putNumber("TurretRotateEncoder", TurretThetaActual);
    SmartDashboard.putNumber("TurretThetaTarget", TurretThetaTarget);
    SmartDashboard.putNumber("TurretRotateMotorPower", s_TurretRotateMotor.get());

    // Hood Numbers
    SmartDashboard.putNumber("TurretHoodEncoder", HoodThetaActual);
    SmartDashboard.putNumber("HoodThetaTarget", HoodThetaTarget);
    SmartDashboard.putNumber("TurretHoodMotorPower", s_HoodTiltMotor.get());

    // Shooter Numbers
    SmartDashboard.putNumber("ShooterVelocityEncoder", ShooterVelocityActual);
    SmartDashboard.putNumber("ShooterVelocityTarget", ShooterVelocityTarget);
    SmartDashboard.getNumber("ShooterLeftMotorVelocity", s_ShooterMotorLeft.get());

    /*
     * Publishes numbers to Advantage Scope that can be adjusted without the need for editing robot code.
     * NOT SUITABLE FOR MATCH CODE
     */
    // Turret Tuning Values
    SmartDashboard.putData(TurretRotatePID);
    SmartDashboard.getNumber("TurretRotateScoreOffset", Constants.TurretSubsystemConstants.TurretRotateScoreOffset);
    
    // Hood Tuning Values
    SmartDashboard.putData(TurretHoodPID);

    // Shooter Tuning Values
    SmartDashboard.putData(ShooterPID);
    SmartDashboard.putNumber("DistanceToGoal", DistanceToGoal);
  }
}
