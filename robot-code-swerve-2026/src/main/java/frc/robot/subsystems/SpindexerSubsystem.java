// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
  /** Creates a new SpindexerSubsystem. */

  SparkFlex SpindexerMotor = new SparkFlex(Constants.SpindexerSubsystemConstants.SpindexerCANID, MotorType.kBrushless);
  private RelativeEncoder SpindexerEncoder;

  PIDController SpindexerPID = new PIDController(0.0002, 0, 0.00001);
  SimpleMotorFeedforward SpindexerFeedForward = new SimpleMotorFeedforward(0.0, 0.00016);

  double SpindexerVelocityTarget = 2500;
  double SpindexerVelocityActual;
  double SpindexerMotorPower;

  public SpindexerSubsystem() {
    SpindexerMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("SpindexerPID", SpindexerPID);
    SpindexerEncoder = SpindexerMotor.getEncoder();
    SpindexerVelocityActual = SpindexerEncoder.getVelocity();
    SmartDashboard.putNumber("SpindexerVelocityActual", SpindexerVelocityActual);
    SpindexerMotorPower = SpindexerPID.calculate(SpindexerVelocityActual, SpindexerVelocityTarget) + SpindexerFeedForward.calculate(SpindexerVelocityTarget);

  }

  public void SpindexerForward() {
    SpindexerMotor.set(SpindexerMotorPower);
  }

  public void SpindexerReverse() {
    SpindexerMotor.set(-0.25);
  }

  public void SpindexerStop() {
    SpindexerMotor.stopMotor();
  }
}
