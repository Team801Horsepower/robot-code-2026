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

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  SparkFlex FeederMotor = new SparkFlex(Constants.FeederSubsystemConstants.FeederCANID, MotorType.kBrushless);
  private RelativeEncoder FeederEncoder;

  PIDController FeederPID = new PIDController(0.00017, 0, 0.000004);
  SimpleMotorFeedforward FeederFeedForward = new SimpleMotorFeedforward(0.0, 0.00016);

  double FeederVelocityTarget = 5167.5;
  double FeederVelocityActual;
  double FeederMotorPower;

  public FeederSubsystem() {
    FeederMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("FeederPID", FeederPID);
    FeederEncoder = FeederMotor.getEncoder();
    FeederVelocityActual = FeederEncoder.getVelocity();
    SmartDashboard.putNumber("FeederEncoder", FeederVelocityActual);
    FeederMotorPower = FeederPID.calculate(FeederVelocityActual, FeederVelocityTarget) + FeederFeedForward.calculate(FeederVelocityTarget);
  }

  public void Feed() {
    FeederMotor.set(FeederMotorPower);
  }
  public void FeederStop() {
    FeederMotor.stopMotor();
  }

}
