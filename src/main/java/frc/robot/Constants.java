// Copyright (c) 2026 Team 801 Horsepower
package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Robot-wide constants shared across subsystems and commands.
 *
 * <p>Grouping constants by inner class keeps this file manageable and avoids name collisions.
 *
 * <p><b>How to read units in this file:</b>
 * <ul>
 *   <li>Distances: meters (m)
 *   <li>Angles: radians (rad) unless noted
 *   <li>Speeds: meters per second (m/s) or radians per second (rad/s)
 * </ul>
 */
public final class Constants {

  // Prevent instantiation
  private Constants() {}

  // ─── Controller / OI ───────────────────────────────────────────────────────

  /** Xbox controller port numbers as configured in the DS. */
  public static final class OperatorConstants {
    /** Driver controller – USB port 0. Left stick = translation, Right stick = rotation. */
    public static final int kDriverControllerPort = 0;

    /**
     * Joystick deadband applied before any math is done on stick values. Prevents drift from a
     * stick that doesn't quite return to center.
     */
    public static final double kDeadbandDriver = 0.08;
  }

  // ─── QuestNav ──────────────────────────────────────────────────────────────

  // ─── Gather ────────────────────────────────────────────────────────────────

  public static final class GatherConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) powering the gatherer roller. */
    public static final int kMotorOneID = 23;
    public static final int kMotorTwoID = 22;
    /** Default roller power when gathering. Positive = intake direction. */
    public static final double kDefaultPower = 1.0;
    /** Full power for reverse (ejection) intake. Independently configurable. */
    public static final double kReverseIntakePower = 0.7;
  }

  // ─── Hopper ────────────────────────────────────────────────────────────────

  public static final class HopperConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) driving the hopper rail. */
    public static final int kMotorId = 27;
    /** DIO channel A for the REV Through Bore Encoder (quadrature). */
    public static final int kEncoderDioA = 1;
    /** DIO channel B for the REV Through Bore Encoder (quadrature). */
    public static final int kEncoderDioB = 3;
    /**
     * Fully-extended motor encoder position (motor rotations from home).
     * Measured with motor encoder zeroed at full retraction.
     */
    public static final double kExtendedSetpoint = 15.801;
    /**
     * Partial extension motor encoder position (motor rotations from home).
     * Measured with motor encoder zeroed at full retraction.
     */
    public static final double kPartialExtendSetpoint = 23.245955;
    /** Closed-loop position tolerance (motor rotations). */
    public static final double kTolerance = 0.1;

    // Position thresholds for extended/retracted checks (motor rotations).
    public static final double kExtendMinPosition = 15.4;
    public static final double kExtendMaxPosition = 16.0;
    public static final double kRetractMinPosition = 1;
    public static final double kRetractMaxPosition = 2;

    // PID gains for hopper extension
    public static final double kExtendP = 0.1;
    public static final double kExtendI = 0;
    public static final double kExtendD = 0;
    // PID gains for hopper retraction
    public static final double kRetractP = 0.1;
    public static final double kRetractI = 0;
    public static final double kRetractD = 0;

    /** @deprecated Use kExtendP instead. Kept for SmartDashboard compatibility. */
    @Deprecated
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  // ─── Climb ─────────────────────────────────────────────────────────────

  public static final class ClimbConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) powering the climb mechanism. */
    public static final int kMotorId = 29;
    /** Fully-climbed motor encoder position (motor rotations from rest). Placeholder — tune on robot. */
    public static final double kClimbedSetpoint = -125.0;
    /** Fully extended setpoint */
    public static final double kExtendSetpoint = -216.75;
    /** Climber Climb PID Values */
    public static final double kClimbP = 0.05;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    //** Climber Reset PID Values */
    public static final double kResetP = 0.05;
    public static final double kResetI = 0;
    public static final double kResetD = 0;
  }

  // ─── Spindex ───────────────────────────────────────────────────────────────

  public static final class SpindexConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) spinning the spindexer. */
    public static final int kMotorId = 25;
    /** Target spindexer velocity (RPM). */
    public static final double kTargetVelocityRPM = 6500.0;
    /** Velocity PID P gain. */
    public static final double kVelocityP = 0.0002;
    /** Velocity PID I gain. */
    public static final double kVelocityI = 0.0;
    /** Velocity PID D gain. */
    public static final double kVelocityD = 0.00001;
    /** Velocity feedforward kS */
    public static final double kVelocityFFkS = 0.0;
    /** Velocity feedforward kV */
    public static final double kVelocityFFkV = 0.00016;
  }

  // ─── Feeder ────────────────────────────────────────────────────────────────

  public static final class FeederConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) powering the feeder. */
    public static final int kMotorId = 24;
    /** Target feeder velocity (RPM). */
    public static final double kTargetVelocityRPM = 5167.5;
    /** Velocity PID P gain. */
    public static final double kVelocityP = 0.00017;
    /** Velocity PID I gain. */
    public static final double kVelocityI = 0.0;
    /** Velocity PID D gain. */
    public static final double kVelocityD = 0.000004;
    /** Velocity feedforward kS */
    public static final double kVelocityFFkS = 0.0;
    /** Velocity feedforward kV */
    public static final double kVelocityFFkV = 0.00016;

  }

  // ─── Drive ─────────────────────────────────────────────────────────────────

  /** Teleop drive speed limits. */
  public static final class DriveConstants {
    /**
     * Maximum translational speed during teleop. Tune this value to match your drive characterisation
     * or set it to the theoretical free-speed of your drivetrain.
     *
     * <p>Kraken X60 free-speed ~6000 RPM; with a 6.75:1 ratio and 4" wheels ≈ 5.4 m/s. Derate
     * slightly for safety.
     */
    public static final double kMaxSpeedMetersPerSecond = 5.0; // m/s

    /**
     * Maximum rotational speed during teleop. Full stick = one full rotation per ~1.4 seconds
     * which is comfortable to drive.
     */
    public static final double kMaxAngularSpeedRadPerSec = Math.PI * 1.5; // rad/s (~1.5π rad/s)

    /** Drive speed multiplier applied when right trigger > kShootSlowdownThreshold. */
    public static final double kShootWhileMovingSpeedMultiplier = 0.40;

    /** Right trigger axis value at which the speed multiplier activates. */
    public static final double kShootSlowdownThreshold = 0.15;
  }
  public static final class QuestSubsystemConstants {
    // Starting Position 1 - Testing
    public static final double RobotStart1TestX = 3.536352; // added 0.0254 thrice
    public static final double RobotStart1TestY = 3.812136; // subtracted 0.2032
    public static final double RobotStart1TestZ = 0.0;
    public static final double RobotStart1TestRoll = 0.0;
    public static final double RobotStart1TestPitch = 0.0;
    public static final double RobotStart1TestYaw = Math.PI;

    // Quest to Robot Orientation
    public static final double QuestToRobotX = -0.263;
    public static final double QuestToRobotY = 0.229;
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
    // Blue Alliance TargetPositions
    public static final Pose3d BlueAllianceGoal = new Pose3d(4.635, 4.034, 1.8288, new Rotation3d(0,0,0));
    public static final Pose3d AimPointB1 = new Pose3d(3.977894, 5.759196, 1.8288, new Rotation3d(0,0,0));
    public static final Pose3d AimPointB2 = new Pose3d(3.977894, 2.31013, 1.8288, new Rotation3d(0,0,0));

    // Red Alliance Goal TargetPositions
    public static final Pose3d RedAllianceGoal = new Pose3d(11.9068, 4.034, 1.8288, new Rotation3d(0,0,0));
    public static final Pose3d AimPointR1 = new Pose3d(12.563094, 2.31013, 1.8288, new Rotation3d(0,0,0));
    public static final Pose3d AimPointR2 = new Pose3d(12.563094, 5.759196, 1.8288, new Rotation3d(0,0,0));
    
    // Robot To Turret
    public static final double RobotToTurretX = -0.103165;
    public static final double RobotToTurretY = -0.094050;
    public static final double RobotToTurretZ = 0.5588; // 22 inches
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
    public static double TurretRotateScoreOffset = -0.797;

    /** Multiplier on turret lead offset. 0 = disabled, 1 = full compensation. */
    public static final double kLeadFactor = 1.0;

    /** Multiplier on radial velocity correction. 0 = disabled, 1 = full compensation. */
    public static final double kRadialVelocityFactor = 1.0;

    /** Baseline vertical distance from turret to goal on flat ground (meters). */
    public static final double BaselineDeltaZ = 1.27; // GoalHeight(1.8288) - TurretHeight(0.5588)
  }

  // ─── Field ──────────────────────────────────────────────────────────────────

  public enum FieldZone { LAUNCH, TRENCH, FAR }

  public static final class FieldConstants {
    /** Field length in meters (54 ft 1 in). */
    public static final double kFieldLengthMeters = 16.5418;
    /** Field width in meters (26 ft 7.25 in). */
    public static final double kFieldWidthMeters = 8.0518;

    /** End of launch zone, measured from friendly wall (156.61 inches). */
    public static final double kLaunchZoneEndMeters = 3.978;
    /** End of trench zone, measured from friendly wall (201.01 inches). */
    public static final double kTrenchZoneEndMeters = 5.106;

    /**
     * Returns the field zone the robot is currently in.
     *
     * @param robotX Robot X position in meters (WPILib field coordinates)
     * @param alliance Current alliance color
     * @return The field zone
     */
    public static FieldZone getFieldZone(double robotX, edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
      double distFromFriendlyWall;
      if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
        distFromFriendlyWall = kFieldLengthMeters - robotX;
      } else {
        distFromFriendlyWall = robotX;
      }

      if (distFromFriendlyWall <= kLaunchZoneEndMeters) {
        return FieldZone.LAUNCH;
      } else if (distFromFriendlyWall <= kTrenchZoneEndMeters) {
        return FieldZone.TRENCH;
      } else {
        return FieldZone.FAR;
      }
    }
  }

}
