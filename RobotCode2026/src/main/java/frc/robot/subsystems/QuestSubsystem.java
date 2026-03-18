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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SendableChooser<Pose3d> RobotStartingPositionChooser = new SendableChooser<>();

  /** Creates a new QuestSubsystem. */
  public QuestSubsystem() {
    RobotStartingPositionChooser.addOption("BlueFarLeft", Constants.FieldPositioningConstants.RobotStartBlueFarLeft);
    RobotStartingPositionChooser.addOption("BlueLeft", Constants.FieldPositioningConstants.RobotStartBlueLeft);
    RobotStartingPositionChooser.addOption("BlueRight", Constants.FieldPositioningConstants.RobotStartBlueRight);
    RobotStartingPositionChooser.addOption("BlueFarRight", Constants.FieldPositioningConstants.RobotStartBlueFarRight);
    RobotStartingPositionChooser.addOption("RedFarLeft", Constants.FieldPositioningConstants.RobotStartRedFarLeft);
    RobotStartingPositionChooser.addOption("RedLeft", Constants.FieldPositioningConstants.RobotStartRedLeft);
    RobotStartingPositionChooser.addOption("RedRight", Constants.FieldPositioningConstants.RobotStartRedRight);
    RobotStartingPositionChooser.addOption("RedFarRight", Constants.FieldPositioningConstants.RobotStartRedFarRight);
    SmartDashboard.putData("RobotStartingPosition", RobotStartingPositionChooser);

    RobotPose = RobotStartingPositionChooser.getSelected();
    QuestPose = RobotPose.transformBy(QuestToRobot.inverse());
    questNav.setPose(QuestPose);;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : poseFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questNav.isTracking()) {
                // Get the pose of the Quest
                QuestPose = questFrame.questPose3d();
                RobotPose = QuestPose.transformBy(QuestToRobot);

                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();
            }
    }

  publisher.set(RobotPose.toPose2d());
  }
}
