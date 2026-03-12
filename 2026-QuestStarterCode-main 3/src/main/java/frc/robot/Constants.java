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
  public static final class CANIDConstants {
    public static final int GatherMototCANID = 12;
    public static final int SpindexerMotorCANID = 21;
    public static final int FeederMotorCANID = 22;
    public static final int FlywheelMotorLeftCANID = 25;
    public static final int FlywheelMotorRightCANID = 26;
    public static final int TurretRotateMotorCANID = 23;
    public static final int HoodTiltMotorCANID = 24;
    public static final int TurretRotateEncoderDIOID = 0;
  }
  public static final class FeedConstants {
    public static final double GatherOuttakeSpeed = -0.50;
    public static final double SpindexerForwardSpeed = 0.50;
    public static final double SpindexerReverseSpeed = 0.50;
    public static final double FeederSpeed = 0.50;
    public static final double ShooterWheelSpeed = 0.50;
    public static final double ShooterVelocityEfficiency = 0.5;
  }
  public static final class RedAllianceGoalPositionConstants{
    public static final double GoalX = 1.0;
    public static final double GoalY = 1.0;
    public static final double GoalZ = 1.0;
  }
  public static final class BlueAllianceGoalPositionConstants{
    public static final double GoalX = 1.0;
    public static final double GoalY = 1.0;
    public static final double GoalZ = 1.0;
  }
  public static final class RobotStartingPositionConstants{
    public static final double RobotStartX = 1.0;
    public static final double RobotStartY = 1.0;
    public static final double RobotStartZ = 1.0;
    public static final double RobotStartRotation = 1.0;
  }
  public static final class RobotConstants{
    public static final double HoodGearRatio = 26.25;
  }
}
