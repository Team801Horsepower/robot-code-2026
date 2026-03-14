// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class QuestSubsystemConstants {
    // Starting Position 1 - Testing
    public static final double RobotStart1TestX = 3.383952;
    public static final double RobotStart1TestY = 4.015336;
    public static final double RobotStart1TestZ = 0.0;
    public static final double RobotStart1TestRoll = 0.0;
    public static final double RobotStart1TestPitch = 0.0;
    public static final double RobotStart1TestYaw = 3.14159;

    // Quest to Robot Orientation
    public static final double QuestToRobotX = 0.0;
    public static final double QuestToRobotY = 0.0;
    public static final double QuestToRobotZ = 0.0;
    public static final double QuestToRobotRoll = 0.0;
    public static final double QuestToRobotPitch = 0.0;
    public static final double QuestToRobotYaw = 3.14159;
    
  }

  public static final class TurretSubsystemConstants{
    // CANIDS
    public static final int FlywheelMotorLeftCANID = 20;
    public static final int FlywheelMotorRightCANID = 21;
    public static final int TurretRotateMotorCANID = 28;
    public static final int HoodTiltMotorCANID = 26;
    public static final int TurretRotateEncoderDIOID = 0;

    // Blue Alliance Goal Position
    public static final double BlueGoalX = 4.635;
    public static final double BlueGoalY = 4.034;
    public static final double BlueGoalZ = 1.0;

    // Red Alliance Goal Position
    public static final double RedGoalX = 1.0;
    public static final double RedGoalY = 1.0;
    public static final double RedGoalZ = 1.0;
    
    // Quest To Turret
    public static final double QuestToTurretX = 0.193556;
    public static final double QuestToTurretY = -0.328754;

    // Measurements
    public static final double HoodGearRatio = 26.25;
    public static final double ShooterWheelCircumference = 0.2934;

    // Turret Limits
    public static final double TurretRotateFreedom = 7.33038;
    public static final double TurretRotateOffset = 3.66519;

    // Tuning Constants
    public static double ShooterVelcoityEfficiency = 0.5;
    public static double ShooterVelocityMultiplier = 1.0;
    public static double TurretRotateScoreOffset = 0.0;
  }
}
