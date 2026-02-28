// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;



public class QuestNavSubsystem extends SubsystemBase {

    
    
    private final QuestNav questNav = new QuestNav();
    private final CommandSwerveDrivetrain swerveDriveSubsystem;
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
        );

    /** Transform from robot center frame to Quest headset frame. Configure for your mount. */
    private static final Transform3d ROBOT_TO_QUEST = new Transform3d(new Translation3d(0.195, 0.023, 0.219), new Rotation3d());
    // Field and NT entries to publish transformed robot pose for AdvantageScope
    private final Field2d m_field = new Field2d();
    private final NetworkTable m_qnTable = NetworkTableInstance.getDefault().getTable("QuestNav");
    // Yaw smoothing settings for displayed Field2d (0=no smoothing, 1=freeze)
    private static final double YAW_SMOOTHING_ALPHA = 0.25; // lower = less sensitive (more smoothing)
    private double m_filteredYawRad = 0.5;
    private boolean m_hasFilteredYaw = false;


    /**
     * Construct the QuestNavSubsystem.
     * @param swerveDriveSubsystem The drivetrain subsystem to call addVisionMeasurement(...) on
     */
    public QuestNavSubsystem(CommandSwerveDrivetrain swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    @Override
    public void periodic() {
    // Run QuestNav periodic processing (required by the vendor library)
    questNav.commandPeriodic();

    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());


                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator

                // Publish transformed robot pose for AdvantageScope / NetworkTables
                // Publish as a 2D pose [x, y, yaw_deg]
                Pose2d robotPose2d = robotPose.toPose2d();

                // Smooth yaw for display so the field marker is less sensitive/jittery
                double rawYaw = robotPose2d.getRotation().getRadians();
                if (!m_hasFilteredYaw) {
                    m_filteredYawRad = rawYaw;
                    m_hasFilteredYaw = true;
                } else {
                    // compute shortest signed angle difference
                    double delta = Math.atan2(Math.sin(rawYaw - m_filteredYawRad), Math.cos(rawYaw - m_filteredYawRad));
                    m_filteredYawRad += YAW_SMOOTHING_ALPHA * delta;
                }

                Pose2d displayPose = new Pose2d(robotPose2d.getX(), robotPose2d.getY(), new Rotation2d(m_filteredYawRad));
                // Update Field2d so AdvantageScope / dashboard can draw it with smoothed yaw
                m_field.setRobotPose(displayPose);
                SmartDashboard.putData("QuestNav Field", m_field);
            }
        }
    }
}