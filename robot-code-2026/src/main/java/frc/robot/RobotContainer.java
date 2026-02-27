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
import frc.robot.TunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;

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
 *   <li><b>Start button</b> – Reset QuestNav heading to "forward = current direction" (re-zero)
 *   <li><b>Back button</b> – Reset robot pose to origin (0, 0, 0°) for testing
 * </ul>
 *
 * <h2>Field-centric driving explained</h2>
 * In field-centric mode, "forward" (positive left-stick Y) always pushes the robot toward the
 * far end of the field regardless of which direction the robot is facing. The robot's heading
 * from QuestNav is used to rotate the stick input into field coordinates. This feels much more
 * intuitive to drive.
 */
public class RobotContainer {

  // ─── Subsystems ────────────────────────────────────────────────────────────

  /**
   * The swerve drivetrain subsystem. Created via factory method so TunerConstants can supply
   * the correct CTRE module constants.
   */
  private final DrivetrainSubsystem m_drivetrain = TunerConstants.createDrivetrain();

  /**
   * The QuestNav subsystem. Must be created after the drivetrain because it takes a reference to
   * it so it can inject vision measurements.
   */
  private final QuestNavSubsystem m_questNav = new QuestNavSubsystem(m_drivetrain);

  // ─── Controllers ───────────────────────────────────────────────────────────

  /** Primary driver controller wired to USB port 0. */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // ─── Swerve Requests ───────────────────────────────────────────────────────

  /**
   * Field-centric drive request. "Field-centric" means positive X always points in the same
   * direction on the field regardless of robot orientation. CTRE uses this internally together with
   * the pose-estimator heading (which is dominated by QuestNav after vision measurements).
   *
   * <p>We use {@link DriveRequestType#OpenLoopVoltage} for driver feel during teleop. Switch to
   * {@link DriveRequestType#Velocity} if you want closed-loop velocity control.
   */
  private final SwerveRequest.FieldCentric m_fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(DriveConstants.kMaxSpeedMetersPerSecond * 0.02)
          .withRotationalDeadband(DriveConstants.kMaxAngularSpeedRadPerSec * 0.02)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * Used briefly while the robot is braking / locking wheels (X-stance). Triggers when left
   * trigger is held.
   */
  private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

  // ─── Constructor ───────────────────────────────────────────────────────────

  /**
   * Constructs RobotContainer: sets up subsystem default commands and controller button bindings.
   */
  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
  }

  // ─── Default Commands ──────────────────────────────────────────────────────

  /**
   * Configures the default command for the drivetrain.
   *
   * <p>The default command runs whenever no other command requiring the drivetrain is scheduled.
   * Here it drives the robot field-centrically from the driver controller.
   */
  private void configureDefaultCommands() {
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() -> buildFieldCentricRequest()));
  }

  /**
   * Builds a {@link SwerveRequest.FieldCentric} populated with the current driver stick values.
   *
   * <p>Stick inputs are:
   * <ul>
   *   <li>Left stick Y → forward/backward translation (note: Y-axis on gamepads is inverted –
   *       pulling stick toward you is positive, but the robot should go forward, so we negate)
   *   <li>Left stick X → left/right strafe
   *   <li>Right stick X → rotation (negate: pushing right = CW = negative in WPILib)
   * </ul>
   */
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

  /**
   * Wires controller buttons to commands.
   *
   * <p>All bindings are configured here so they can be found quickly in one place.
   */
  private void configureButtonBindings() {

    // Left trigger (held) → lock wheels in X-stance (defensive brake)
    m_driverController
        .leftTrigger(0.5)
        .whileTrue(m_drivetrain.applyRequest(() -> m_brakeRequest));

    // Start button → re-zero QuestNav heading so the robot's current direction becomes "forward"
    // on the field. Useful after bumping the headset or if the operator perspective is off.
    m_driverController
        .start()
        .onTrue(Commands.runOnce(() -> {
          // Tell CTRE to treat the robot's current direction as the "operator forward" direction.
          // This does NOT move the robot – only changes which way the driver's stick pushes it.
          m_drivetrain.seedFieldRelative();
        }));

    // Back button → reset pose to (0, 0, 0°) – useful for debugging at the start of the match
    // when the robot is placed at the origin.
    m_driverController
        .back()
        .onTrue(Commands.runOnce(() ->
            m_questNav.resetPoseToFieldPosition(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))));

    // Y button → re-seed the drivetrain heading from QuestNav directly (if tracking)
    m_driverController
        .y()
        .onTrue(Commands.runOnce(() -> {
          if (m_questNav.isTracking()) {
            m_drivetrain.seedPose(m_questNav.getLatestRobotPose2d());
          }
        }));
  }

  // ─── Autonomous ────────────────────────────────────────────────────────────

  /**
   * Returns the autonomous command to run during the 15-second auto period.
   *
   * <p>Currently returns a placeholder that prints a message. Replace this with your PathPlanner
   * or Choreo autonomous routine when you're ready.
   *
   * @return The autonomous command
   */
  public Command getAutonomousCommand() {
    // TODO: Replace with your actual autonomous command / path planner routine
    return Commands.print("[Auto] No autonomous command configured!");
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  /**
   * Applies a deadband to a joystick axis value.
   *
   * <p>Values within ±{@link OperatorConstants#kDeadbandDriver} of zero are snapped to zero. This
   * prevents the robot from drifting due to joystick centering imperfection.
   *
   * @param input Raw joystick axis value in [-1, 1]
   * @return Deadbanded value in [-1, 1]
   */
  private static double applyDeadband(double input) {
    return MathUtil.applyDeadband(input, OperatorConstants.kDeadbandDriver);
  }
}