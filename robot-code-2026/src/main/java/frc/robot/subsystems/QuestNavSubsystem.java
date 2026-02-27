// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import frc.robot.Constants.QuestNavConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

/**
 * QuestNavSubsystem – Integrates the Meta Quest 3 headset as the robot's primary pose / heading
 * source using the QuestNav vendor library.
 *
 * <h2>How it works</h2>
 * <ol>
 *   <li>The Quest headset runs the QuestNav app which publishes its 6-DOF pose over NT4.
 *   <li>Each periodic loop, this subsystem reads all new {@link PoseFrame}s from the Quest.
 *   <li>Each frame is transformed from Quest coordinates to robot-centre coordinates using the
 *       {@link QuestNavConstants#ROBOT_TO_QUEST} offset.
 *   <li>The resulting {@link Pose3d} is fed into the CTRE swerve pose estimator via
 *       {@link DrivetrainSubsystem#addVisionMeasurement} with tight standard deviations, so
 *       <b>QuestNav effectively becomes the primary heading source</b>.
 * </ol>
 *
 * <h2>QuestNav coordinate system vs. WPILib</h2>
 * The QuestNav vendor library already converts the Quest's Unity coordinate system to the WPILib
 * NWU (North-West-Up) coordinate system, so no additional axis flipping is required here.
 *
 * <h2>Heading convention</h2>
 * QuestNav's yaw is CCW-positive in the WPILib frame. Wherever the code references "heading" it
 * means this yaw angle.
 */
public class QuestNavSubsystem extends SubsystemBase {

  // ─── Hardware ──────────────────────────────────────────────────────────────

  /**
   * The QuestNav vendor-library object. Manages the NT4 connection to the headset and decodes
   * incoming {@link PoseFrame} protobuf messages.
   */
  private final QuestNav m_questNav = new QuestNav();

  // ─── References to other subsystems ───────────────────────────────────────

  /** The drivetrain subsystem that will receive QuestNav vision measurements. */
  private final DrivetrainSubsystem m_drivetrain;

  // ─── Vision measurement trust ──────────────────────────────────────────────

  /**
   * Standard deviations for QuestNav measurements fed into the CTRE pose estimator.
   *
   * <p>Lower = more trust. These values (2 cm, 2 cm, ~2 degrees) make QuestNav the dominant
   * heading source and suppress Pigeon 2 drift.
   */
  private static final Matrix<N3, N1> kQuestStdDevs =
      VecBuilder.fill(
          QuestNavConstants.kStdDevX,
          QuestNavConstants.kStdDevY,
          QuestNavConstants.kStdDevTheta);

  // ─── State ─────────────────────────────────────────────────────────────────

  /** The last valid robot pose3d received from QuestNav. May be null before first packet. */
  private Pose3d m_lastPose3d = null;

  /** True when the Quest is connected and actively tracking. */
  private boolean m_isTracking = false;

  // ─── NetworkTables debug publishers ────────────────────────────────────────

  /**
   * Publishes the raw Quest pose (not transformed) for debugging in AdvantageScope.
   * AdvantageScope path: {@code /QuestNav/RawQuestPose3d}.
   */
  private final DoubleArrayPublisher m_rawPosePublisher;

  /**
   * Publishes the transformed robot pose (what gets sent to the drivetrain estimator).
   * AdvantageScope path: {@code /QuestNav/RobotPose3d}.
   */
  private final DoubleArrayPublisher m_robotPosePublisher;

  /** Publishes the current heading (yaw) in degrees for easy dashboard reading. */
  private final DoublePublisher m_headingDegPublisher;

  /** Publishes whether QuestNav is actively tracking as a boolean. */
  private final BooleanPublisher m_trackingPublisher;

  // ─── Constructor ───────────────────────────────────────────────────────────

  /**
   * Constructs the QuestNavSubsystem.
   *
   * @param drivetrain The {@link DrivetrainSubsystem} that QuestNav will update
   */
  public QuestNavSubsystem(DrivetrainSubsystem drivetrain) {
    this.m_drivetrain = drivetrain;

    // ── NetworkTables publishers ──────────────────────────────────────────────
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table = nt.getTable("QuestNav");

    m_rawPosePublisher    = table.getDoubleArrayTopic("RawQuestPose3d").publish();
    m_robotPosePublisher  = table.getDoubleArrayTopic("RobotPose3d").publish();
    m_headingDegPublisher = table.getDoubleTopic("HeadingDegrees").publish();
    m_trackingPublisher   = table.getBooleanTopic("IsTracking").publish();
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  /**
   * Called every 20 ms by the CommandScheduler.
   *
   * <ol>
   *   <li>Calls {@link QuestNav#commandPeriodic()} – required by QuestNav vendor library.
   *   <li>Reads all new pose frames since the last loop.
   *   <li>For each valid frame, transforms to robot pose and feeds the drivetrain estimator.
   *   <li>Publishes debug data to NetworkTables.
   * </ol>
   */
  @Override
  public void periodic() {
    // ── REQUIRED: must be called every periodic loop ─────────────────────────
    m_questNav.commandPeriodic();

    // ── Read all unread pose frames ────────────────────────────────────────────
    PoseFrame[] frames = m_questNav.getAllUnreadPoseFrames();

    boolean anyTracking = false;

    for (PoseFrame frame : frames) {
      if (!frame.isTracking()) {
        continue; // Skip frames where Quest lost tracking
      }

      anyTracking = true;

      // Raw Quest pose (in WPILib coordinate frame, already converted by the library)
      Pose3d questPose = frame.questPose3d();

      // Transform from Quest-mounting-point coordinates to robot-center coordinates.
      // We use the inverse of ROBOT_TO_QUEST: going from Quest pose → robot pose.
      Pose3d robotPose3d = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

      // Timestamp of when the data was captured on the Quest (seconds, FPGA epoch)
      double timestamp = frame.dataTimestamp();

      // Feed into the CTRE swerve pose estimator.
      // With tight std devs this effectively replaces Pigeon heading with QuestNav heading.
      m_drivetrain.addVisionMeasurement(robotPose3d.toPose2d(), timestamp, kQuestStdDevs);

      // Cache the last valid pose for callers who just want the current pose
      m_lastPose3d = robotPose3d;

      // ── Debug publishers ───────────────────────────────────────────────────
      publishPose3d(m_rawPosePublisher, questPose);
      publishPose3d(m_robotPosePublisher, robotPose3d);
    }

    m_isTracking = anyTracking;

    // ── Heading + status to dashboard ─────────────────────────────────────────
    m_trackingPublisher.set(m_isTracking);

    if (m_lastPose3d != null) {
      double headingDeg = Math.toDegrees(m_lastPose3d.getRotation().getZ());
      m_headingDegPublisher.set(headingDeg);
      SmartDashboard.putNumber("QuestNav/Heading (deg)", headingDeg);
    }

    SmartDashboard.putBoolean("QuestNav/Tracking", m_isTracking);
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Returns true when the Quest headset is connected and actively tracking.
   *
   * <p>Always check this before trusting QuestNav pose data in auto routines.
   *
   * @return {@code true} if tracking
   */
  public boolean isTracking() {
    return m_isTracking;
  }

  /**
   * Returns the last known robot {@link Pose3d} derived from QuestNav, or {@code null} if no data
   * has been received yet.
   *
   * @return Latest robot pose, or {@code null}
   */
  public Pose3d getLatestRobotPose3d() {
    return m_lastPose3d;
  }

  /**
   * Returns the last known robot {@link Pose2d}, or {@code null} if no data has been received.
   *
   * @return Latest 2-D pose, or {@code null}
   */
  public Pose2d getLatestRobotPose2d() {
    return m_lastPose3d != null ? m_lastPose3d.toPose2d() : null;
  }

  /**
   * Returns the current heading (yaw) from QuestNav in radians, CCW-positive.
   *
   * <p>Returns {@code 0.0} if no pose data is available yet.
   *
   * @return Heading in radians
   */
  public double getHeadingRadians() {
    if (m_lastPose3d == null) return 0.0;
    return m_lastPose3d.getRotation().getZ();
  }

  /**
   * Returns the current heading from QuestNav as a {@link Rotation2d}.
   *
   * @return Heading rotation
   */
  public Rotation2d getHeading() {
    return new Rotation2d(getHeadingRadians());
  }

  /**
   * Commands the QuestNav system to reset its internal pose to a known field-relative pose.
   *
   * <p>Use this at the start of autonomous to align QuestNav's origin with a known starting
   * position on the field.
   *
   * <p>Internally converts the robot pose back to Quest coordinates before sending.
   *
   * @param robotPose The desired field-relative robot pose
   */
  public void resetPoseToFieldPosition(Pose2d robotPose) {
    // Convert robot pose → Quest pose (apply the mount transform forward, not inverse)
    Pose3d questPose = new Pose3d(robotPose).transformBy(QuestNavConstants.ROBOT_TO_QUEST);
    m_questNav.setPose(questPose);

    // Also seed the CTRE estimator immediately so driving is field-correct right away
    m_drivetrain.seedPose(robotPose);
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  /**
   * Serialises a {@link Pose3d} into the double[] format expected by AdvantageScope's 3-D field
   * widget: {@code [x, y, z, roll, pitch, yaw]} (all in meters and radians).
   */
  private void publishPose3d(DoubleArrayPublisher publisher, Pose3d pose) {
    publisher.set(new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(), // roll
        pose.getRotation().getY(), // pitch
        pose.getRotation().getZ()  // yaw
    });
  }
}