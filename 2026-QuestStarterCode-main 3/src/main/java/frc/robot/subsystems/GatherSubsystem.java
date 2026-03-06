// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class GatherSubsystem extends SubsystemBase {
  /** Creates a new GatherSubsystem. */
  SparkFlex s_GatherMotor = new SparkFlex(Constants.CANIDConstants.GatherMototCANID, MotorType.kBrushless);

  public GatherSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TriggerAxisValue", RobotContainer.driverController.getLeftTriggerAxis());
    // This method will be called once per scheduler run
  }

  public void RunGatherIntake(double speed){
    if (Math.abs(speed) > 0.1) {
        s_GatherMotor.set(speed);
    }
    else {
      Stop();
    }
  }
  public void RunGatherOuttake() {
    s_GatherMotor.set(Constants.FeedConstants.GatherOuttakeSpeed);
  }

  public void Stop(){
    s_GatherMotor.stopMotor();
  }

  public double GetSpeed(){
    return s_GatherMotor.get();
  }
}
