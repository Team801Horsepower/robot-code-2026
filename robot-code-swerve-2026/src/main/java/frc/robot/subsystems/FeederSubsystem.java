// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  SparkFlex FeederMotor = new SparkFlex(Constants.FeederSubsystemConstants.FeederCANID, MotorType.kBrushless);

  public FeederSubsystem() {
    FeederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Shoot() {
    FeederMotor.set(.75);
  }
  public void FeederStop() {
    FeederMotor.stopMotor();
  }

}
