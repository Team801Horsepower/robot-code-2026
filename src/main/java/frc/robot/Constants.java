// Copyright (c) 2026 Team 801 Horsepower
package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


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

  /** Constants for the QuestNav VR-headset odometry subsystem. */
  public static final class QuestNavConstants {

    /**
     * Rigid-body transform from the <b>robot center</b> (floor level, centre of the drivebase) to
     * the Quest headset's tracking origin.
     *
     * <p>Measure these offsets on the physical robot and update accordingly:
     * <ul>
     *   <li>X: forward/backward from robot centre (+ = forward)
     *   <li>Y: left/right from robot centre (+ = left)
     *   <li>Z: height above ground (+ = up)
     *   <li>Rotation: how the Quest is rotated relative to the robot's forward direction
     * </ul>
     *
     * <p><b>TODO:</b> Measure and fill in your actual Quest mount offsets!
     */
    public static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(
            new Translation3d(
                0.0, // X offset (meters) – forward from robot centre
                0.0, // Y offset (meters) – left from robot centre
                0.5), // Z offset (meters) – height above ground
            new Rotation3d(
                0.0, // Roll  (rad)
                0.0, // Pitch (rad)
                0.0  // Yaw   (rad) – 0 means Quest faces same direction as robot front
            ));

    /**
     * Standard deviations (trust values) for QuestNav pose measurements fed into the CTRE
     * SwerveDriveState pose estimator.
     *
     * <p>Lower values = more trust in QuestNav. Values in [X meters, Y meters, θ radians]. At
     * 0.02 m and ~0.035 rad we trust QuestNav to within ~2 cm and ~2 degrees.
     */
    public static final double kStdDevX = 0.02;
    public static final double kStdDevY = 0.02;
    public static final double kStdDevTheta = 0.035;
  }

  // ─── Gather ────────────────────────────────────────────────────────────────

  public static final class GatherConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) powering the gatherer roller. */
    public static final int kMotorId = 12;
    /** Default roller power when gathering. Positive = intake direction. */
    public static final double kDefaultPower = 0.7;
    /** Full power for reverse (ejection) intake. Independently configurable. */
    public static final double kReverseIntakePower = 0.7;
  }

  // ─── Hopper ────────────────────────────────────────────────────────────────

  public static final class HopperConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) driving the hopper rail. */
    public static final int kMotorId = 11;
    /**
     * Fully-extended encoder position (motor rotations from home).
     * TODO: Take into account that one rotation of the encoder's gear equals 2.1 linear inches.
     */
    public static final double kExtendedSetpoint = 50.0;
    /** Closed-loop position tolerance (rotations). */
    public static final double kTolerance = 0.5;
    // PID gains for hopper position control. TODO: tune.
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  // ─── Spindex ───────────────────────────────────────────────────────────────

  public static final class SpindexConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) spinning the spindexer. */
    public static final int kMotorId = 21;
    /** Power applied during spin() (positive; motor is set negative in code). */
    public static final double kSpinPower = 0.8;
    /** Waveform shape for agitate() and Hopper.jostle(). */
    public static final AgitationType kAgitationType = AgitationType.SINUSOIDAL;
    /**
     * Agitation amplitude.
     * - For Spindex: maps directly to motor speed (0–1).
     * - For Hopper jostle: MUST be ≤ 0.5 (positions 1−amplitude to 1−2·amplitude).
     */
    public static final double kAmplitude = 0.3;
    /** Agitation period in seconds. */
    public static final double kPeriod = 1.0;
    /**
     * Reversed flag.
     * false → zero is the minimum power (center at +amplitude).
     * true  → zero is the center (oscillates ±amplitude around zero).
     */
    public static final boolean kAgitationReversed = false;
  }

  // ─── Feeder ────────────────────────────────────────────────────────────────

  public static final class FeederConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) powering the feeder. */
    public static final int kMotorId = 22;
    /** Default feeder power. */
    public static final double kSpinPower = 1.0;
  }

  // ─── Turret ────────────────────────────────────────────────────────────────

  public static final class TurretConstants {
    // CAN IDs
    public static final int kLaunchMotor1Id = 25;
    public static final int kLaunchMotor2Id = 26;
    public static final int kHoodMotorId    = 24;
    public static final int kRotateMotorId  = 23;

    /** Target launch-wheel velocity (RPM). TODO: tune. */
    public static final double kTargetVelocityRPM = 4000.0;
    /** Velocity PID P gain. TODO: tune. */
    public static final double kVelocityP = 0.001;
    /** Velocity feedforward (fraction per RPM). TODO: tune. */
    public static final double kVelocityFF = 0.00022;

    /** Power applied when rotating the turret to a target angle. */
    public static final double kRotatePower = 0.3;
    /** Turret rotation dead-band (degrees). */
    public static final double kRotateToleranceDeg = 2.0;
    /**
     * Maximum turret rotation in either direction (degrees).
     * Full range = 2 × 210 = 420°, mapped to one encoder revolution.
     */
    public static final double kMaxRotationDeg = 210.0;
    /**
     * Absolute-encoder position conversion factor so that getPosition() returns turret degrees.
     * One full encoder revolution = 420° of turret travel.
     */
    public static final double kRotateEncoderConversionFactor = 420.0;

    /** Minimum hood angle (degrees). Physical resting position. */
    public static final double kHoodMinDeg = 15.0;
    /** Maximum useful hood angle (degrees). */
    public static final double kHoodMaxDeg = 30.0;
    /** Absolute physical travel limit of the hood (degrees equivalent). TODO: measure. */
    public static final double kHoodMaxEncoderPos = 15.0; // encoder rotations at 30° – TODO tune
    /** Hardware safety limit for the hood encoder (encoder rotations). TODO: measure. */
    public static final double kHoodPhysicalMaxEncoderPos = 30.0;
    /** Power applied when moving the hood. */
    public static final double kAimPower = 0.3;
    /** Hood position dead-band (encoder rotations). */
    public static final double kAimToleranceEncoder = 0.5;
  }

  // ─── Climber ───────────────────────────────────────────────────────────────

  public static final class ClimberConstants {
    /** CAN ID of the NEO Vortex (SparkFlex) driving the climber slide. */
    public static final int kMotorId = 31;
    /**
     * Encoder position (motor rotations) when the climber is fully extended to L1.
     * TODO: measure on physical robot.
     */
    public static final double kExtendedSetpoint = 100.0;
    /** Tolerance for considering the climber "at setpoint" (rotations). */
    public static final double kTolerance = 1.0;
    /** Power applied while driving the climber up or down. */
    public static final double kMotorPower = 0.5;
    // ElevatorFeedforward gains. TODO: tune (kG must overcome gravity to hold position).
    public static final double kFeedforwardKs = 0.0;
    public static final double kFeedforwardKg = 0.05;
    public static final double kFeedforwardKv = 0.0;
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
}