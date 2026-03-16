// Copyright (c) 2026 Team 801 Horsepower
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GatherConstants;
import frc.robot.Constants.HopperConstants;
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
 *   <li><b>D-pad Down</b> – Toggle jostle type (cycles FLAT → SINUSOIDAL → ABSOLUTE_VALUE)
 * </ul>
 */
public class RobotContainer {

  // ─── Leaf subsystems ───────────────────────────────────────────────────────

  private final DrivetrainSubsystem m_drivetrain = TunerConstants.createDrivetrain();
  private final QuestSubsystem m_QuestSubsystem = new QuestSubsystem();
  private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(m_QuestSubsystem);

  private final Gather   m_gather  = new Gather();
  private final Hopper   m_hopper  = new Hopper();
  private final Spindex  m_spindex = new Spindex();
  private final Feeder   m_feeder  = new Feeder();

  // ─── Composite subsystems ──────────────────────────────────────────────────

  private final Drive       m_drive      = new Drive(m_drivetrain);
  private final Possession  m_possession = new Possession(m_hopper, m_gather);
  private final Launch      m_launch     = new Launch(m_spindex, m_feeder, m_gather);
  private final Manipulator m_manipulator = new Manipulator(m_possession, m_launch);

  // ─── Controllers ───────────────────────────────────────────────────────────

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private boolean m_complexMode = false;

  private final EventLoop m_testLoop = new EventLoop();

  // ─── Auto chooser ─────────────────────────────────────────────────────────

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

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
    configureAutoChooser();
    SmartDashboard.putBoolean("Complex Mode", m_complexMode);
  }

  // ─── Default Commands ──────────────────────────────────────────────────────

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        m_drive.applyRequest(() -> buildFieldCentricRequest()));
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
        .whileTrue(new Shoot(m_launch, m_hopper, () -> m_complexMode));

    // Right bumper (held) → jostle to unjam
    m_driverController
        .rightBumper()
        .whileTrue(new Jostling(m_spindex));

    // A button → toggle complex launch mode
    m_driverController
        .a()
        .onTrue(Commands.runOnce(() -> {
            m_complexMode = !m_complexMode;
            SmartDashboard.putBoolean("Complex Mode", m_complexMode);
        }));

    // X button → retract hopper
    m_driverController
        .x()
        .onTrue(Commands.runOnce(() -> m_hopper.retract(), m_hopper));

    // D-pad Down → cycle jostle type
    m_driverController
        .povDown()
        .onTrue(Commands.runOnce(() -> m_spindex.cycleAgitationType()));
  }

  // ─── Test Mode Bindings ─────────────────────────────────────────────────────

  private void configureTestBindings() {
    // Left trigger (held) → Gather forward (+0.5)
    m_driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.08, m_testLoop)
        .whileTrue(Commands.run(() -> m_gather.testRun(0.5), m_gather));

    // Left bumper (held) → Gather reverse (-0.5)
    m_driverController.button(XboxController.Button.kLeftBumper.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_gather.testRun(-0.5), m_gather));

    // Right trigger (held) → Feeder forward (+0.75)
    m_driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.15, m_testLoop)
        .whileTrue(Commands.run(() -> m_feeder.testRun(0.75), m_feeder));

    // D-pad Down (held) → Feeder reverse (-0.75)
    m_driverController.pov(0, 180, m_testLoop)
        .whileTrue(Commands.run(() -> m_feeder.testRun(-0.75), m_feeder));

    // Right bumper (held) → Spindex (+0.5)
    m_driverController.button(XboxController.Button.kRightBumper.value, m_testLoop)
        .whileTrue(Commands.run(() -> m_spindex.testRun(0.5), m_spindex));

    // D-pad Left → Hopper retract (PID to 0)
    m_driverController.pov(0, 270, m_testLoop)
        .onTrue(Commands.runOnce(() -> m_hopper.testSetPosition(0), m_hopper));

    // D-pad Right → Hopper full extend (PID to kExtendedSetpoint)
    m_driverController.pov(0, 90, m_testLoop)
        .onTrue(Commands.runOnce(
            () -> m_hopper.testSetPosition(HopperConstants.kExtendedSetpoint), m_hopper));

    // A → Hopper partial extend (PID to kPartialExtendSetpoint)
    m_driverController.button(XboxController.Button.kA.value, m_testLoop)
        .onTrue(Commands.runOnce(
            () -> m_hopper.testSetPosition(HopperConstants.kPartialExtendSetpoint), m_hopper));

    // NOTE: Turret (B, X, Y, D-pad Up) handled via polling default command
    // WARNING: D-pad Left/Right and A overlap with turret polling — both subsystems will respond
  }

  // ─── Mode Switching ───────────────────────────────────────────────────────────

  public void configureTestMode() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setActiveButtonLoop(m_testLoop);

    m_gather.setTestMode(true);
    m_hopper.setTestMode(true);
    m_spindex.setTestMode(true);
    m_feeder.setTestMode(true);
    m_TurretSubsystem.setTestMode(true);

    // Drive: same field-centric controls as teleop
    m_drive.setDefaultCommand(
        m_drive.applyRequest(() -> buildFieldCentricRequest()));

    // Simple subsystems: idle at 0 (trigger bindings override when held)
    m_gather.setDefaultCommand(Commands.run(() -> m_gather.testRun(0), m_gather));
    // Hopper: no-op default — PID holds last commanded position from test bindings
    m_hopper.setDefaultCommand(Commands.run(() -> {}, m_hopper));
    m_spindex.setDefaultCommand(Commands.run(() -> m_spindex.testRun(0), m_spindex));
    m_feeder.setDefaultCommand(Commands.run(() -> m_feeder.testRun(0), m_feeder));
    // Turret: polling default command (reads HID directly for 3 motor groups)
    // Y = launch, B = hood+, D-pad Up = hood-, X = rotate+, D-pad Down = rotate-
    m_TurretSubsystem.setDefaultCommand(Commands.run(() -> {
      var hid = m_driverController.getHID();

      double launchPower = hid.getYButton() ? 0.5 : 0;
      double hoodPower   = hid.getBButton() ? 0.05 : (hid.getPOV() == 0 ? -0.05 : 0);
      double rotatePower = hid.getXButton() ? 0.1 : (hid.getPOV() == 180 ? -0.1 : 0);

      m_TurretSubsystem.testRunLaunch(launchPower);
      m_TurretSubsystem.testRunHood(hoodPower);
      m_TurretSubsystem.testRunRotate(rotatePower);
    }, m_TurretSubsystem));
  }

  public void configureNormalMode() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setActiveButtonLoop(
        CommandScheduler.getInstance().getDefaultButtonLoop());

    m_gather.setTestMode(false);
    m_hopper.setTestMode(false);
    m_spindex.setTestMode(false);
    m_feeder.setTestMode(false);
    m_TurretSubsystem.setTestMode(false);

    configureDefaultCommands();
  }

  // ─── Auto Chooser ─────────────────────────────────────────────────────────

  private void configureAutoChooser() {
    m_autoChooser.setDefaultOption("None", Commands.none());

    m_autoChooser.addOption("CH", Commands.none());
    m_autoChooser.addOption("CH-Prime", Commands.none());
    m_autoChooser.addOption("FLS", Commands.none());
    m_autoChooser.addOption("FRS", Commands.none());
    m_autoChooser.addOption("LS", Commands.none());
    m_autoChooser.addOption("RS", Commands.none());

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  private static double applyDeadband(double input) {
    return MathUtil.applyDeadband(input, OperatorConstants.kDeadbandDriver);
  }
}
