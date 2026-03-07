// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  SparkFlex s_FlywheelMotorLeft = new SparkFlex(Constants.CANIDConstants.FlywheelMotorLeftCANID, MotorType.kBrushless);
  SparkFlex s_FlywheelMotorRight = new SparkFlex(Constants.CANIDConstants.FlywheelMotorRightCANID, MotorType.kBrushless);
  SparkFlex s_TurretRotateMotor = new SparkFlex(Constants.CANIDConstants.TurretRotateMotorCANID, MotorType.kBrushless);
  SparkFlex s_HoodTiltMotor = new SparkFlex(Constants.CANIDConstants.HoodTiltMotorCANID, MotorType.kBrushless);

  CANcoder s_TurretRotateEncoder = new CANcoder(Constants.CANIDConstants.TurretRotateEncoderCANID);
  RelativeEncoder s_TurretHoodEncoder = s_HoodTiltMotor.getEncoder();

  PIDController TurretRotatePID = new PIDController(0, 0, 0);
  PIDController TurretHoodPID = new PIDController(0, 0, 0);

  // Vector Variables
  public double vX;
  public double vY;
  public double vZ;

  // Robot Position Variables
  public double RobotX;
  public double RobotY;
  public double RobotZ;
  public double RobotYaw;

  //Turret Variables
  public double TurretTheta;
  public double HoodTheta;
  public double TurretEncoderValue;

  public TurretSubsystem() {
    s_TurretHoodEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    vX = Constants.GoalPositionConstants.GoalX - RobotX;
    vY = Constants.GoalPositionConstants.GoalY - RobotY;
    vZ = Constants.GoalPositionConstants.GoalZ - RobotZ;
    
    TurretTheta = Math.tan(vX/vY) + RobotYaw;

    HoodTheta = Math.tan(vZ/(Math.sqrt(vX*vX + vY*vY)));

    s_TurretRotateMotor.set(TurretRotatePID.calculate(TurretEncoderValue, TurretTheta));
  }
}
