// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
  /** Creates a new SpindexerSubsystem. */
SparkFlex s_SpindexerMotor = new SparkFlex(Constants.CANIDConstants.SpindexerMotorCANID, MotorType.kBrushless);

  public SpindexerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RunSpindexerForward() {
    s_SpindexerMotor.set(Constants.FeedConstants.SpindexerForwardSpeed);
  }

  public void RunSpindexerReverse(double speed) {
    s_SpindexerMotor.set(Constants.FeedConstants.SpindexerReverseSpeed);
  }

  public void Stop () {
    s_SpindexerMotor.stopMotor();
  }
}
