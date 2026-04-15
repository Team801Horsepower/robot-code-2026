// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestSubsystem extends SubsystemBase {
  QuestNav questNav = new QuestNav();

  public Pose3d RobotPose = new Pose3d();
  public Pose3d QuestPose = new Pose3d();

  Transform3d QuestToRobot = new Transform3d(
    Constants.QuestSubsystemConstants.QuestToRobotX, 
    Constants.QuestSubsystemConstants.QuestToRobotY, 
    Constants.QuestSubsystemConstants.QuestToRobotZ, 
    new Rotation3d(
      Constants.QuestSubsystemConstants.QuestToRobotRoll, 
      Constants.QuestSubsystemConstants.QuestToRobotPitch, 
      Constants.QuestSubsystemConstants.QuestToRobotYaw));

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  /** Creates a new QuestSubsystem. */
  public QuestSubsystem() {}

  /**
   * Seeds QuestNav with a known robot pose. Call at the start of autonomous
   * to tell QuestNav where the robot has been placed on the field.
   *
   * @param pose The robot's known starting pose
   */
  public void setPose(Pose3d pose) {
    Pose3d questPose = pose.transformBy(QuestToRobot.inverse());
    questNav.setPose(questPose);
  }

  /** Returns true if QuestNav is actively tracking the headset pose. */
  public boolean isTracking() {
    return questNav.isTracking();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    refreshPose();
  }

  /**
   * Drains any unread Quest pose frames and updates {@link #QuestPose} / {@link #RobotPose}
   * to the newest one. Safe (and cheap) to call multiple times per loop — if no frames have
   * arrived since the last call, this is a no-op. Consumers that need the freshest possible
   * pose (e.g. TurretSubsystem) can call this at the top of their own periodic to avoid the
   * one-loop lag from subsystem-periodic ordering.
   */
  public void refreshPose() {
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : poseFrames) {
      if (questNav.isTracking()) {
        QuestPose = questFrame.questPose3d();
        RobotPose = QuestPose.transformBy(QuestToRobot);
      }

      publisher.set(RobotPose.toPose2d());
    }
  }

  public Pose2d getPose2d() {
    return QuestPose.toPose2d();
  }
}
