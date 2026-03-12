// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  SparkFlex s_FlywheelMotorLeft = new SparkFlex(Constants.CANIDConstants.FlywheelMotorLeftCANID, MotorType.kBrushless);
  SparkFlex s_FlywheelMotorRight = new SparkFlex(Constants.CANIDConstants.FlywheelMotorRightCANID, MotorType.kBrushless);
  SparkFlex s_TurretRotateMotor = new SparkFlex(Constants.CANIDConstants.TurretRotateMotorCANID, MotorType.kBrushless);
  SparkFlex s_HoodTiltMotor = new SparkFlex(Constants.CANIDConstants.HoodTiltMotorCANID, MotorType.kBrushless);

  DutyCycleEncoder s_TurretRotateEncoder = new DutyCycleEncoder(Constants.CANIDConstants.TurretRotateEncoderDIOID, 420, 0);
  RelativeEncoder s_TurretHoodEncoder = s_HoodTiltMotor.getEncoder();

  PIDController TurretRotatePID = new PIDController(0, 0, 0);
  ArmFeedforward TurretRotateFeedForward = new ArmFeedforward(0, 0, 0);
  PIDController TurretHoodPID = new PIDController(0, 0, 0);
  ArmFeedforward TurretHoodFeedForward = new ArmFeedforward(0, 0, 0);
  PIDController FlyWheelPID = new PIDController(0, 0, 0);
  SimpleMotorFeedforward FlyWheelFeedForward = new SimpleMotorFeedforward(0, 0);

  Pose2d robotPose = new Pose2d();
  Rotation2d RobotRotation = new Rotation2d();

  // Vector Variables
  public double vX;
  public double vY;
  public double vZ;

  // Goal Position Variables
  public double GoalPositionX;
  public double GoalPositionY;
  public double GoalPositionZ;

  // Robot Position Variables
  public double RobotX;
  public double RobotY;
  public double RobotZ;
  public double RobotYaw;

  //Turret Variables
  public double TicksPerDegree;
  public double TurretThetaActual;
  public double TurretThetaTarget;
  public double HoodThetaActual;
  public double HoodThetaTarget;

  public double Velocity;
  public double MotorVelocity;

  public TurretSubsystem() {
    s_TurretHoodEncoder.setPosition(0);

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Blue) {
      GoalPositionX = Constants.BlueAllianceGoalPositionConstants.GoalX;
      GoalPositionY = Constants.BlueAllianceGoalPositionConstants.GoalY;
      GoalPositionZ = Constants.BlueAllianceGoalPositionConstants.GoalZ;
    }
    else if (ally.get() == Alliance.Red) {
      GoalPositionX = Constants.RedAllianceGoalPositionConstants.GoalX;
      GoalPositionY = Constants.RedAllianceGoalPositionConstants.GoalY;
      GoalPositionZ = Constants.RedAllianceGoalPositionConstants.GoalZ;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotX = robotPose.getX() + Constants.RobotStartingPositionConstants.RobotStartX;
    RobotY = robotPose.getY() + Constants.RobotStartingPositionConstants.RobotStartY;
    RobotYaw = RobotRotation.getDegrees();

    vX = GoalPositionX - RobotX;
    vY = GoalPositionY - RobotY;
    vZ = GoalPositionZ - RobotZ;
    
    TurretThetaActual = s_TurretRotateEncoder.get();
    TurretThetaTarget = Math.tan(vX/vY) + RobotYaw;

    TicksPerDegree = (7168 * Constants.RobotConstants.HoodGearRatio) / 360;
    HoodThetaActual = s_TurretHoodEncoder.getPosition() / TicksPerDegree;
    //HoodThetaTarget = Math.tan(vZ/(Math.sqrt(vX*vX + vY*vY)));
    HoodThetaTarget = (0.5) * (Math.asin(Math.sqrt((-9.81 * (Math.sqrt(vX*vX + vY*vY))) / (Velocity * Velocity))));
    MotorVelocity = (Velocity) / (Constants.FeedConstants.ShooterVelocityEfficiency * (3 * Math.PI));
    
    SmartDashboard.getNumber("HoodThetaTarget", HoodThetaTarget);
    SmartDashboard.getNumber("TurretThetaTarget", TurretThetaTarget);
    SmartDashboard.getNumber("TurretRotateEncoderValue", TurretThetaActual);
    SmartDashboard.getNumber("TurretHoodEncoder", HoodThetaActual);
  }
  public void TurretAimDefaultCommand (){

    /*
    RobotX = robotPose.getX() + Constants.RobotStartingPositionConstants.RobotStartX;
    RobotY = robotPose.getY() + Constants.RobotStartingPositionConstants.RobotStartY;
    RobotYaw = RobotRotation.getDegrees();

    vX = GoalPositionX - RobotX;
    vY = GoalPositionY - RobotY;
    vZ = GoalPositionZ - RobotZ;
    
    TurretThetaActual = s_TurretRotateEncoder.get();
    TurretThetaTarget = Math.tan(vX/vY) + RobotYaw;

    TicksPerDegree = (7168 * Constants.RobotConstants.HoodGearRatio) / 360;
    HoodThetaActual = s_TurretHoodEncoder.getPosition() / TicksPerDegree;
    //HoodThetaTarget = Math.tan(vZ/(Math.sqrt(vX*vX + vY*vY)));
    HoodThetaTarget = (0.5) * (Math.asin(Math.sqrt((-9.81 * (Math.sqrt(vX*vX + vY*vY))) / (Velocity * Velocity))));
    MotorVelocity = (Velocity) / (Constants.FeedConstants.ShooterVelocityEfficiency * (3 * Math.PI));
    
    SmartDashboard.getNumber("HoodThetaTarget", HoodThetaTarget);
    SmartDashboard.getNumber("TurretThetaTarget", TurretThetaTarget);
    SmartDashboard.getNumber("TurretRotateEncoderValue", TurretThetaActual);
    SmartDashboard.getNumber("TurretHoodEncoder", HoodThetaActual);
    */

    // Turret Rotation Control (Feedback + Feedforward)
    //TurretRotatePID.enableContinuousInput(HoodThetaActual = 0, HoodThetaActual = 420);
    //s_TurretRotateMotor.setVoltage(
      //(TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) + 
      //(TurretRotateFeedForward.calculate(TurretThetaTarget,0)));

    // Hood Rotatation Control (Feedback + Feedforward)
    

  }
}
