// Copyright (c) 2026 Team 801 Horsepower
package frc.robot;


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
    public static final int kMotorId = 23;
    /** Default roller power when gathering. Positive = intake direction. */
    public static final double kDefaultPower = 0.7;
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
    public static final double kExtendedSetpoint = 28.183872;
    /**
     * Partial extension motor encoder position (motor rotations from home).
     * Measured with motor encoder zeroed at full retraction.
     */
    public static final double kPartialExtendSetpoint = 23.245955;
    /** Closed-loop position tolerance (motor rotations). */
    public static final double kTolerance = 0.1;
    // PID gains for hopper position control. TODO: tune.
    public static final double kP = 0.6;
    public static final double kI = 0.003;
    public static final double kD = 0.0;

  }

  // ─── Spindex ───────────────────────────────────────────────────────────────

  public static final class SpindexConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) spinning the spindexer. */
    public static final int kMotorId = 25;
    /** Target spindexer velocity (RPM). */
    public static final double kTargetVelocityRPM = 2500.0;
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
  }
  public static final class QuestSubsystemConstants {
    // Starting Position 1 - Testing
    public static final double RobotStart1TestX = 3.460152;
    public static final double RobotStart1TestY = 4.015336;
    public static final double RobotStart1TestZ = 0.0;
    public static final double RobotStart1TestRoll = 0.0;
    public static final double RobotStart1TestPitch = 0.0;
    public static final double RobotStart1TestYaw = 3.14159;

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

    // Blue Alliance Goal Position
    public static final double BlueGoalX = 4.635;
    public static final double BlueGoalY = 4.034;
    public static final double BlueGoalZ = 1.0;

    // Red Alliance Goal Position (field-mirrored from blue; only X flipped)
    public static final double RedGoalX = 11.9068;
    public static final double RedGoalY = 4.034;
    public static final double RedGoalZ = 1.0;
    
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

  // ─── Autonomous ─────────────────────────────────────────────────────────────

  public static final class AutoConstants {
    // Starting positions extracted from PathPlanner path first waypoint anchors.
    // These are robot-center poses (PathPlanner uses robot-center coordinates).

    // Center start (linked name "Center" in C-H1.path, C-D1.path)
    public static final double kCenterStartX = 3.586;
    public static final double kCenterStartY = 4.067;
    public static final double kCenterStartYaw = Math.PI; // 180 degrees

    // Right start (linked name "Right" in R-S1.path, R-C1.path)
    public static final double kRightStartX = 3.648;
    public static final double kRightStartY = 2.210;
    public static final double kRightStartYaw = 0.0;

    // Left start (linked name "Left" in L-S1.path)
    public static final double kLeftStartX = 3.637;
    public static final double kLeftStartY = 5.794;
    public static final double kLeftStartYaw = Math.PI / 2.0; // 90 degrees

    // Far Right start (linked name "Far Right" in FR-S1.path)
    public static final double kFarRightStartX = 4.414;
    public static final double kFarRightStartY = 0.432;
    public static final double kFarRightStartYaw = 0.0;

    // Far Left start (linked name "Far Left" in FL-S1.path)
    public static final double kFarLeftStartX = 4.401;
    public static final double kFarLeftStartY = 7.625;
    public static final double kFarLeftStartYaw = 0.0;
  }
}