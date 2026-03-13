// Copyright (c) 2026 Team 801 Horsepower
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GatherConstants;
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
 * <h2>Single controller (Xbox Controller – Port 0)</h2>
 * <ul>
 *   <li><b>Left stick X/Y</b> – Translational velocity (field-relative)
 *   <li><b>Right stick X</b> – Rotational velocity
 *   <li><b>Left trigger (&gt;0.08, held)</b> – Scaled intake (trigger axis = gather power)
 *   <li><b>Left bumper (held)</b> – Reverse intake (full configurable power)
 *   <li><b>Right trigger (&gt;0.15, held)</b> – Shoot (run launch sequence)
 *   <li><b>Right bumper (held)</b> – Jostle (unjam spindexer + hopper)
 *   <li><b>D-pad Up</b> – Climb to Level 1
 *   <li><b>D-pad Down</b> – Toggle jostle type (cycles FLAT → SINUSOIDAL → ABSOLUTE_VALUE)
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
  private final Launch      m_launch     = new Launch(m_spindex, m_feeder, m_turret, m_gather);
  private final Ascension   m_ascension  = new Ascension(m_level1);
  private final Manipulator m_manipulator = new Manipulator(m_possession, m_launch);
  private final Vision      m_vision     = new Vision(m_questNav, m_drivetrain);

  // ─── Controllers ───────────────────────────────────────────────────────────

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final EventLoop m_testLoop = new EventLoop();

  // ─── Swerve requests ───────────────────────────────────────────────────────

  private final SwerveRequest.FieldCentric m_fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(DriveConstants.kMaxSpeedMetersPerSecond * 0.02)
          .withRotationalDeadband(DriveConstants.kMaxAngularSpeedRadPerSec * 0.02)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // ─── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureTestBindings();
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

    // Left trigger (>0.08, held) → scaled intake (trigger axis = gather power)
    m_driverController
        .leftTrigger(0.08)
        .whileTrue(new Gathering(m_possession,
            () -> m_driverController.getLeftTriggerAxis()));

    // Left bumper (held) → reverse intake at configurable full power
    m_driverController
        .leftBumper()
        .whileTrue(Commands.run(
            () -> m_gather.gather(-GatherConstants.kReverseIntakePower), m_gather));

    // Right trigger (>0.15, held) → shoot
    m_driverController
        .rightTrigger(0.15)
        .whileTrue(new Shoot(m_launch, m_hopper));

    // Right bumper (held) → jostle to unjam
    m_driverController
        .rightBumper()
        .whileTrue(new Jostling(m_spindex));

    // D-pad Up → climb to Level 1
    m_driverController
        .povUp()
        .onTrue(new ClimbL1(m_hopper, m_ascension));

    // D-pad Down → cycle jostle type
    m_driverController
        .povDown()
        .onTrue(Commands.runOnce(() -> m_spindex.cycleAgitationType()));
  }

  // ─── Test Mode Bindings ─────────────────────────────────────────────────────

  private void configureTestBindings() {
    // Left trigger (held) → Gather forward (+0.1)
    m_driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.08, m_testLoop)
        .whileTrue(Commands.run(() -> m_gather.testRun(0.1), m_gather));

    // Left bumper (held) → Gather reverse (-0.1)
    m_driverController.button(XboxController.Button.kLeftBumper.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_gather.testRun(-0.1), m_gather));

    // Right trigger (held) → Feeder forward (+0.75)
    m_driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.15, m_testLoop)
        .whileTrue(Commands.run(() -> m_feeder.testRun(0.75), m_feeder));

    // D-pad Down (held) → Feeder reverse (-0.75)
    m_driverController.pov(0, 180, m_testLoop)
        .whileTrue(Commands.run(() -> m_feeder.testRun(-0.75), m_feeder));

    // Right bumper (held) → Spindex (+0.5)
    m_driverController.button(XboxController.Button.kRightBumper.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_spindex.testRun(0.5), m_spindex));

    // Y (held) → Hopper forward (+0.5)
    m_driverController.button(XboxController.Button.kY.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_hopper.testRun(0.5), m_hopper));

    // D-pad Up (held) → Hopper reverse (-0.5)
    m_driverController.pov(0, 0, m_testLoop)
        .whileTrue(Commands.run(() -> m_hopper.testRun(-0.5), m_hopper));

    // Start (held) → Level1 up (+0.1)
    m_driverController.button(XboxController.Button.kStart.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_level1.testRun(0.1), m_level1));

    // Back (held) → Level1 down (-0.1)
    m_driverController.button(XboxController.Button.kBack.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_level1.testRun(-0.1), m_level1));

    // NOTE: Turret (A, B, X, D-pad Left, D-pad Right) handled via polling default command
  }

  // ─── Mode Switching ───────────────────────────────────────────────────────────

  public void configureTestMode() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setActiveButtonLoop(m_testLoop);

    m_gather.setTestMode(true);
    m_hopper.setTestMode(true);
    m_spindex.setTestMode(true);
    m_feeder.setTestMode(true);
    m_turret.setTestMode(true);
    m_level1.setTestMode(true);

    // Drive: same field-centric controls as teleop
    m_drive.setDefaultCommand(
        m_drive.applyRequest(() -> buildFieldCentricRequest()));

    // Simple subsystems: idle at 0 (trigger bindings override when held)
    m_gather.setDefaultCommand(Commands.run(() -> m_gather.testRun(0), m_gather));
    m_hopper.setDefaultCommand(Commands.run(() -> m_hopper.testRun(0), m_hopper));
    m_spindex.setDefaultCommand(Commands.run(() -> m_spindex.testRun(0), m_spindex));
    m_feeder.setDefaultCommand(Commands.run(() -> m_feeder.testRun(0), m_feeder));
    m_level1.setDefaultCommand(Commands.run(() -> m_level1.testRun(0), m_level1));

    // Turret: polling default command (reads HID directly for 3 motor groups)
    m_turret.setDefaultCommand(Commands.run(() -> {
      var hid = m_driverController.getHID();

      double launchPower = hid.getAButton() ? 0.4 : 0;
      double hoodPower   = hid.getBButton() ? 0.05 : (hid.getPOV() == 90 ? -0.05 : 0);
      double rotatePower = hid.getXButton() ? 0.1 : (hid.getPOV() == 270 ? -0.1 : 0);

      m_turret.testRunLaunch(launchPower);
      m_turret.testRunHood(hoodPower);
      m_turret.testRunRotate(rotatePower);
    }, m_turret));
  }

  public void configureNormalMode() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setActiveButtonLoop(
        CommandScheduler.getInstance().getDefaultButtonLoop());

    m_gather.setTestMode(false);
    m_hopper.setTestMode(false);
    m_spindex.setTestMode(false);
    m_feeder.setTestMode(false);
    m_turret.setTestMode(false);
    m_level1.setTestMode(false);

    configureDefaultCommands();
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
