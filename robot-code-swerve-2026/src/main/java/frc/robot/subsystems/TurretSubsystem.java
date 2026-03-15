// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;

public class TurretSubsystem extends SubsystemBase {

  SparkFlex s_FlywheelMotorLeft = new SparkFlex(Constants.TurretSubsystemConstants.FlywheelMotorLeftCANID, MotorType.kBrushless);
  SparkFlex s_FlywheelMotorRight = new SparkFlex(Constants.TurretSubsystemConstants.FlywheelMotorRightCANID, MotorType.kBrushless);
  SparkFlex s_TurretRotateMotor = new SparkFlex(Constants.TurretSubsystemConstants.TurretRotateMotorCANID, MotorType.kBrushless);
  SparkFlex s_HoodTiltMotor = new SparkFlex(Constants.TurretSubsystemConstants.HoodTiltMotorCANID, MotorType.kBrushless);

  DutyCycleEncoder s_TurretRotateEncoder = new DutyCycleEncoder(Constants.TurretSubsystemConstants.TurretRotateEncoderDIOID, Constants.TurretSubsystemConstants.TurretRotateFreedom, 0);
  private RelativeEncoder HoodEncoder;
  private RelativeEncoder ShooterEncoder;

  PIDController TurretRotatePID = new PIDController(1.5, 0, 0);
  SimpleMotorFeedforward TurretRotateFeedForward = new SimpleMotorFeedforward(0, 0);
  PIDController TurretHoodPID = new PIDController(0.9, 0.003, 0);
  SimpleMotorFeedforward TurretHoodFeedForward = new SimpleMotorFeedforward(0, 0, 0);
  PIDController FlyWheelPID = new PIDController(0, 0, 0);
  SimpleMotorFeedforward FlyWheelFeedForward = new SimpleMotorFeedforward(0, 0);
  
  QuestSubsystem questNav = new QuestSubsystem();

  // Vector Variables
  public double vX;
  public double vY;

  // Robot Position Variables
  public double TurretX;
  public double TurretY;
  public double TurretZ;
  public double RobotYaw;

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

  public TurretSubsystem() {
    s_HoodTiltMotor.getEncoder().setPosition(0);

    SparkFlexConfig GlobalConfig = new SparkFlexConfig();
    SparkFlexConfig ShooterRightFollowerConfig = new SparkFlexConfig();
    ShooterRightFollowerConfig.inverted(true).apply((GlobalConfig).follow(s_FlywheelMotorRight));

    SmartDashboard.putNumber("ShooterVelocityMultiplier", Constants.TurretSubsystemConstants.ShooterVelocityMultiplier);
    SmartDashboard.putNumber("ShooterVelocityEficiencey", Constants.TurretSubsystemConstants.ShooterVelcoityEfficiency);
    SmartDashboard.putNumber("TurretRotateScoreOffset", Constants.TurretSubsystemConstants.TurretRotateScoreOffset);
  }

  @Override
  public void periodic() {
    /*
     * Takes robot pose2d published by QuestNav (Position of Quest, NOT position of center of robot)
     * and offsets it to the center of the turret.
     */
    TurretX = questNav.RobotPose.getX() + Constants.TurretSubsystemConstants.QuestToTurretX;
    TurretY = questNav.RobotPose.getY() - Constants.TurretSubsystemConstants.QuestToTurretY;
    Rotation3d PoseRotation = questNav.RobotPose.getRotation();
    RobotYaw = PoseRotation.getZ();

    /*
     * Creates a 2D unit vector from the robot to the goal.
     * Calculates distance to goal.
     */
    vX = Constants.TurretSubsystemConstants.BlueGoalX - TurretX;
    vY = Constants.TurretSubsystemConstants.BlueGoalY - TurretY;
    
    DistanceToGoal = Math.sqrt(vX*vX + vY*vY);

    /*
     * TURRET ROTATE
     * Takes the tanget of the 2D unit vector to get the heading of the goal relative the robot.
     * Subtracts the position of the turret from the rotation of the robot to get the true angle of the turret.
     * Feedforward helps to overcome system resistance.
     * Feedback drives turret motor to target turret rotate theta.
     */
    TurretThetaActual = (s_TurretRotateEncoder.get()) - Constants.TurretSubsystemConstants.TurretRotateOffset;
    TurretThetaTarget = MathUtil.clamp(((-Math.atan2(vY, vX) + (RobotYaw)) + 
    Constants.TurretSubsystemConstants.TurretRotateScoreOffset), -3.49066, 3.49066
    );

    s_TurretRotateMotor.set(
    (TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) +
    (TurretRotateFeedForward.calculate(0))
    );
    
    /*
     * TURRET HOOD
     * Takes the encoder value from the Spark Flex built in relative encoder and converts it to hood position in radians.
     * Calculates the hood theta target using the line of best fit from an analysis of several physics equations.
     * Feedforward helps to overcome system resistance.
     * Feedback drives turret hood motor to target hood theta.
     */
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
    HoodThetaTarget = MathUtil.clamp(
      (0.233 + 0.0567 * DistanceToGoal + -0.00485 * Math.pow(DistanceToGoal, 2)),
      0.261799, 0.785398
    );

    s_HoodTiltMotor.set(
      (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
      (TurretHoodFeedForward.calculate(0))
    );

    /*
     * TURRET SHOOTER
     * Calculates the shooter velocity target based on the line of best fit fron an analysis of physics equations.
     * Converts the ball velocity in m/s to motor speed in RPM.
     * Gets the velocity reading from the left flywheel motor.
     * Feedforward drives flywheel to target velocity.
     * Feedback drives flywheel to target velocity. 
     */
    BallVelocityTarget = 4.53 + 1.13 * DistanceToGoal + -0.04 * Math.pow(DistanceToGoal, 2);
    ShooterVelocityTarget = (60 * BallVelocityTarget) / (Constants.TurretSubsystemConstants.ShooterVelcoityEfficiency * Constants.TurretSubsystemConstants.ShooterWheelCircumference);
    ShooterEncoder = s_FlywheelMotorLeft.getEncoder();
    ShooterVelocityActual = ShooterEncoder.getVelocity();

    s_FlywheelMotorLeft.setVoltage( (Constants.TurretSubsystemConstants.ShooterVelocityMultiplier) * 
    (
      (FlyWheelPID.calculate(ShooterVelocityActual, ShooterVelocityTarget)) +
      (FlyWheelFeedForward.calculate(ShooterVelocityTarget))
    )
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
    SmartDashboard.getNumber("ShooterLeftMotorVelocity", s_FlywheelMotorLeft.get());
    SmartDashboard.putNumber("ShooterRightMotorVelocity", s_FlywheelMotorRight.get());

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
    SmartDashboard.putData(FlyWheelPID);
    SmartDashboard.getNumber("ShooterEficiencey", Constants.TurretSubsystemConstants.ShooterVelcoityEfficiency);
    SmartDashboard.getNumber("ShooterVelocityMultiplier", Constants.TurretSubsystemConstants.ShooterVelocityMultiplier);
  }
}
