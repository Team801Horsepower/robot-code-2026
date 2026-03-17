// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GatherSubsystem extends SubsystemBase {
  /** Creates a new GatherSubsystem. */

  SparkFlex GatherMotor = new SparkFlex(Constants.GatherSubsystemConstants.GatherCANID, MotorType.kBrushless);

  public GatherSubsystem() {
    GatherMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void GatherForward() {
    GatherMotor.set(1);
  }

  public void GatherReverse() {
    GatherMotor.set(-1);
  }

  public void StopGather() {
    GatherMotor.stopMotor();
  }
}
