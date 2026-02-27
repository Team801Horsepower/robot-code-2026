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