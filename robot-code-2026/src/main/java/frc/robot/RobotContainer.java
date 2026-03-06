// Copyright (c) 2026 Team 801 Horsepower
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * RobotContainer – the "wiring closet" of the robot.
 *
 * <p>All subsystems are instantiated here. Controller bindings are configured here. The autonomous
 * command is returned from here.
 *
 * <h2>Drive controls (Xbox Controller – Port 0)</h2>
 * <ul>
 *   <li><b>Left stick X/Y</b> – Translational velocity (field-relative)
 *   <li><b>Right stick X</b> – Rotational velocity
 *   <li><b>Left trigger (held)</b> – Wheel lock (X-stance brake)
 *   <li><b>Start button</b> – Reset QuestNav heading to "forward = current direction"
 *   <li><b>Back button</b> – Reset robot pose to origin (0, 0, 0°) for testing
 *   <li><b>Y button</b> – Re-seed drivetrain heading from QuestNav
 * </ul>
 *
 * <h2>Operator controls (Xbox Controller – Port 1)</h2>
 * <ul>
 *   <li><b>Right bumper (held)</b> – Gather (intake game pieces)
 *   <li><b>Right trigger (held)</b> – Shoot (run launch sequence)
 *   <li><b>Left bumper (held)</b> – Jostle (unjam spindexer + hopper)
 *   <li><b>Left trigger (held)</b> – Aim (update turret angle from vision)
 *   <li><b>A button</b> – Climb to Level 1
 * </ul>
 */
public class RobotContainer {

  // ─── Leaf subsystems ───────────────────────────────────────────────────────

  private final DrivetrainSubsystem m_drivetrain = TunerConstants.createDrivetrain();
  private final QuestNavSubsystem   m_questNav   = new QuestNavSubsystem(m_drivetrain);

  private final Gather   m_gather  = new Gather();
  private final Hopper   m_hopper  = new Hopper();
  private final Spindex  m_spindex = new Spindex();
  private final Feeder   m_feeder  = new Feeder();
  private final Turret   m_turret  = new Turret();
  private final Level1   m_level1  = new Level1();

  // ─── Composite subsystems ──────────────────────────────────────────────────

  private final Drive       m_drive      = new Drive(m_drivetrain);
  private final Possession  m_possession = new Possession(m_hopper, m_gather);
  private final Launch      m_launch     = new Launch(m_spindex, m_feeder, m_turret);
  private final Ascension   m_ascension  = new Ascension(m_level1);
  private final Manipulator m_manipulator = new Manipulator(m_possession, m_launch);
  private final Vision      m_vision     = new Vision(m_questNav, m_drivetrain);

  // ─── Controllers ───────────────────────────────────────────────────────────

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(1);

  // ─── Swerve requests ───────────────────────────────────────────────────────

  private final SwerveRequest.FieldCentric m_fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(DriveConstants.kMaxSpeedMetersPerSecond * 0.02)
          .withRotationalDeadband(DriveConstants.kMaxAngularSpeedRadPerSec * 0.02)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

  // ─── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
  }

  // ─── Default Commands ──────────────────────────────────────────────────────

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        m_drive.applyRequest(() -> buildFieldCentricRequest()));
    m_turret.setDefaultCommand(new RunLaunchWheel(m_turret));
  }

  private SwerveRequest.FieldCentric buildFieldCentricRequest() {
    double translationX = applyDeadband(-m_driverController.getLeftY())
        * DriveConstants.kMaxSpeedMetersPerSecond;

    double translationY = applyDeadband(-m_driverController.getLeftX())
        * DriveConstants.kMaxSpeedMetersPerSecond;

    double rotation = applyDeadband(-m_driverController.getRightX())
        * DriveConstants.kMaxAngularSpeedRadPerSec;

    return m_fieldCentricRequest
        .withVelocityX(translationX)
        .withVelocityY(translationY)
        .withRotationalRate(rotation);
  }

  // ─── Button Bindings ───────────────────────────────────────────────────────

  private void configureButtonBindings() {

    // ── Driver controller ─────────────────────────────────────────────────────

    m_driverController
        .leftTrigger(0.5)
        .whileTrue(m_drive.applyRequest(() -> m_brakeRequest));

    m_driverController
        .start()
        .onTrue(Commands.runOnce(() -> m_drive.seedFieldRelative()));

    m_driverController
        .back()
        .onTrue(Commands.runOnce(() ->
            m_questNav.resetPoseToFieldPosition(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))));

    m_driverController
        .y()
        .onTrue(Commands.runOnce(() -> {
          if (m_questNav.isTracking()) {
            m_drive.seedPose(m_questNav.getLatestRobotPose2d());
          }
        }));

    // ── Operator controller ───────────────────────────────────────────────────

    // Right bumper (held) → gather game pieces
    m_operatorController
        .rightBumper()
        .whileTrue(new Gathering(m_possession));

    // Right trigger (held) → shoot
    m_operatorController
        .rightTrigger(0.5)
        .whileTrue(new Shoot(m_launch));

    // Left bumper (held) → jostle to unjam
    m_operatorController
        .leftBumper()
        .whileTrue(new Jostling(m_hopper, m_spindex));

    // Left trigger (held) → aim turret
    m_operatorController
        .leftTrigger(0.5)
        .whileTrue(new Aim(m_vision, m_turret));

    // A button → climb to Level 1
    m_operatorController
        .a()
        .onTrue(new ClimbL1(m_hopper, m_ascension));
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  /**
   * Returns the autonomous command.
   *
   * <p>TODO: Replace with a PathPlanner routine or sequential auto once developed.
   */
  public Command getAutonomousCommand() {
    return Commands.print("[Auto] No autonomous command configured!");
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  private static double applyDeadband(double input) {
    return MathUtil.applyDeadband(input, OperatorConstants.kDeadbandDriver);
  }
}
