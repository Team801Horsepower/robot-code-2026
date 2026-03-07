// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * Aim – continuously updates turret rotation and hood angle based on the robot's pose.
 *
 * <h2>Physics model</h2>
 * <ul>
 *   <li>Target is assumed to be at a known field position (e.g. the speaker/scoring goal).
 *       TODO: update {@code kTargetFieldPos} with the actual 2026 game target position.
 *   <li>Hood angle: simple linear interpolation between min and max angle based on distance.
 *       TODO: replace with a proper ballistic trajectory calculation once the shooter is
 *       characterized.
 *   <li>Turret rotation: bearing from robot to target in robot-relative coordinates.
 * </ul>
 *
 * <p>This command runs indefinitely; cancel it when aiming is no longer needed.
 */
public class Aim extends Command {

  /** Field-relative position of the scoring target (meters). TODO: update for 2026 game. */
  private static final Translation2d kTargetFieldPos = new Translation2d(8.27, 4.01);

  /** Distance at which the minimum hood angle is used (meters). TODO: tune. */
  private static final double kMinAngleDistanceM = 2.0;
  /** Distance at which the maximum hood angle is used (meters). TODO: tune. */
  private static final double kMaxAngleDistanceM = 6.0;

  private final Vision  m_vision;
  private final Turret  m_turret;

  public Aim(Vision vision, Turret turret) {
    m_vision = vision;
    m_turret = turret;
    addRequirements(vision, turret);
  }

  @Override
  public void execute() {
    m_turret.spin(); // maintain wheel velocity while Aim holds the Turret requirement

    Pose2d robotPose = m_vision.report();

    // Vector from robot to target in field coordinates.
    double dx = kTargetFieldPos.getX() - robotPose.getX();
    double dy = kTargetFieldPos.getY() - robotPose.getY();
    double distance = Math.hypot(dx, dy);

    // Turret bearing: angle from robot's forward direction to target (robot-relative, degrees).
    double fieldBearing   = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading   = robotPose.getRotation().getDegrees();
    double robotRelBearing = fieldBearing - robotHeading;
    // Normalize to [-180, 180].
    robotRelBearing = ((robotRelBearing + 180) % 360 + 360) % 360 - 180;

    // Hood angle: linear interpolation from min to max based on distance.
    double clampedDist = Math.max(kMinAngleDistanceM, Math.min(kMaxAngleDistanceM, distance));
    double t = (clampedDist - kMinAngleDistanceM) / (kMaxAngleDistanceM - kMinAngleDistanceM);
    double hoodAngle = TurretConstants.kHoodMinDeg
        + t * (TurretConstants.kHoodMaxDeg - TurretConstants.kHoodMinDeg);

    m_turret.rotate(robotRelBearing);
    m_turret.aim(hoodAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
