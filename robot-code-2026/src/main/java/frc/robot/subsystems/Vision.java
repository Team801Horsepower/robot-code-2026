// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision – aggregates all vision pose sources (QuestNav + PhotonVision) and feeds
 * measurements into the drivetrain pose estimator.
 *
 * <p><b>PhotonVision note:</b> PhotonVision is not yet in the vendordeps. Once added,
 * implement the {@code updatePhotonVision()} method and uncomment the relevant import.
 * TODO: add PhotonVision vendordep and implement camera-based pose estimation.
 *
 * <p>QuestNav already self-updates via {@link QuestNavSubsystem#periodic()}; the
 * {@link #report()} method exposes the latest robot pose for external consumers such
 * as the Aim command.
 */
public class Vision extends SubsystemBase {

  private final QuestNavSubsystem m_questNav;
  private final DrivetrainSubsystem m_drivetrain;

  public Vision(QuestNavSubsystem questNav, DrivetrainSubsystem drivetrain) {
    m_questNav   = questNav;
    m_drivetrain = drivetrain;
  }

  /**
   * Runs all vision sources and returns the latest estimated robot pose.
   *
   * <p>QuestNav measurements are injected automatically via its periodic() loop.
   * This method provides additional triggering and returns the pose for callers
   * that need it (e.g. the Aim command).
   *
   * @return the latest robot pose from the drivetrain's pose estimator
   */
  public Pose2d report() {
    // QuestNav already feeds pose data in QuestNavSubsystem.periodic().
    // TODO: call updatePhotonVision() here when PhotonVision is integrated.
    return m_drivetrain.getPose2d();
  }

  /** Returns whether QuestNav is currently tracking. */
  public boolean isQuestNavTracking() {
    return m_questNav.isTracking();
  }
}
