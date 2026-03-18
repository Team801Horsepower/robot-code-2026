// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

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
  public static final class FieldPositioningConstants {
    /*
     * STARTING POSITIONS FOR THE BLUE ALLIANCE
     */
    //Starting Position Blue Far Left
    public static final Pose3d RobotStartBlueFarLeft = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    //Starting Position Blue Left
    public static final Pose3d RobotStartBlueLeft = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    //Starting Position Blue Right
    public static final Pose3d RobotStartBlueRight = new Pose3d(3.079152,4.015336,0, new Rotation3d(0,0,3.14159));
    //Starting Position Blue Far Right
    public static final Pose3d RobotStartBlueFarRight = new Pose3d(0,0,0, new Rotation3d(0,0,0));

    /*
     * STARTING POSITIONS FOR THE RED ALLIANCE
     */
    //Starting Position Red Far Left
    public static final Pose3d RobotStartRedFarLeft = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    //Starting Position Red Left
    public static final Pose3d RobotStartRedLeft = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    //Starting Position Red Right
    public static final Pose3d RobotStartRedRight = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    //Starting Position Red Far Right
    public static final Pose3d RobotStartRedFarRight = new Pose3d(0,0,0, new Rotation3d(0,0,0));
  }


  public static final class QuestSubsystemConstants {
    // Quest to Robot Orientation
    public static final double QuestToRobotX = -0.296671;
    public static final double QuestToRobotY = 0.234704;
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

    // Blue Alliance TargetPositions
    public static final Pose3d BlueAllianceGoal = new Pose3d(4.635,4.034,0, new Rotation3d(0,0,0));
    public static final Pose3d AimPointB1 = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    public static final Pose3d AimPointB2 = new Pose3d(0,0,0, new Rotation3d(0,0,0));

    // Red Alliance Goal TargetPositions
    public static final Pose3d RedAllianceGoal = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    public static final Pose3d AimPointR1 = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    public static final Pose3d AimPointR2 = new Pose3d(0,0,0, new Rotation3d(0,0,0));
    
    // Robot To Turret
    public static final double RobotToTurretX = -0.103165;
    public static final double RobotToTurretY = -0.094050;
    public static final double RobotToTurretZ = 0.0;
    public static final double RobotToTurretRoll = 0.0;
    public static final double RobotToTurretPitch = 0.0;
    public static final double RobotToTurretYaw = 0.0;

    // Measurements
    public static final double HoodGearRatio = 26.25;
    public static final double ShooterWheelCircumference = 0.2394;

    // Turret Limits
    public static final double TurretRotateFreedom = 7.33038;
    public static final double TurretRotateOffset = 3.66519;

    // Tuning Constants
    public static double ShooterVelcoityEfficiency = 0.5;
    public static double ShooterVelocityMultiplier = 1.0;
    public static double TurretRotateScoreOffset = 0.232;
  }
  public static final class FeederSubsystemConstants{
    public static final int FeederCANID = 24;
  }
  public static final class SpindexerSubsystemConstants{
    public static final int SpindexerCANID = 25;
  }
  public static final class GatherSubsystemConstants {
    public static final int GatherCANID = 23;
  }
  public static final class HopperSubsystemConstants {
    public static final int HopperCANID = 27;
  }
}
