// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  /** Creates a new HopperSubsystem. */

  SparkFlex HopperMotor = new SparkFlex(Constants.HopperSubsystemConstants.HopperCANID, MotorType.kBrushless);
  RelativeEncoder HopperEncoder;

  PIDController HopperExtentionPID = new PIDController(0, 0, 0);
  PIDController HopperRetractionPID = new PIDController(0, 0, 0);

  double HopperExtentionTarget = 27.183872;
  double HopperRetractionTarget = 0.0;
  double HopperExtentionActual;

  double HopperExtendMotorPower;
  double HopperRetractMotorPower;

  double HopperExtendMinPosition = 26;
  double HopperExtendMaxPosition = 27.183872;

  double HopperRetractMinPosition = 0.0;
  double HopperRetractMaxPosition = 1;

  public HopperSubsystem() {
    HopperMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    HopperEncoder = HopperMotor.getEncoder();
    HopperExtentionActual = HopperEncoder.getPosition();

    HopperExtendMotorPower = HopperExtentionPID.calculate(HopperExtentionActual, HopperExtentionTarget);
    HopperRetractMotorPower = HopperRetractionPID.calculate(HopperExtentionActual, HopperRetractionTarget);
    
    SmartDashboard.putData("HopperExtentionPID", HopperExtentionPID);
    SmartDashboard.putData("HopperretractionPID", HopperRetractionPID);
    SmartDashboard.putNumber("Hopper Etention Actual", HopperExtentionActual);
  }

  public void HopperExtend() {
    HopperMotor.set(HopperExtendMotorPower);
  }

  public boolean HopperIsExtended() {
    if (HopperExtentionActual >= HopperExtendMinPosition && HopperExtentionActual <= HopperExtendMaxPosition) {
      return true;
    }
    else {
      return false;
    }
  }

  public void HopperRetract() {
    HopperMotor.set(HopperRetractMotorPower);
  }

  public boolean HopperIsRetracted() {
    if (HopperExtentionActual >= HopperRetractMinPosition && HopperExtentionActual <= HopperRetractMaxPosition) {
      return true;
    }
    else {
      return false;
    }
  }

  public void HopperStop() {
    HopperMotor.stopMotor();
  }
}
