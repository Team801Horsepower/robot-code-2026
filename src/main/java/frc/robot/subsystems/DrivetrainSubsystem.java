// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Team 801's CTRE Phoenix 6 swerve drivetrain subsystem.
 *
 * <p>Extends {@link SwerveDrivetrain} to get the full CTRE swerve stack (high-frequency odometry,
 * pose estimation, module control) and implements WPILib's {@link Subsystem} interface so it
 * participates in the CommandScheduler.
 *
 * <h2>What this class does</h2>
 * <ul>
 *   <li>Creates all four swerve modules from {@link frc.robot.generated.TunerConstants}.
 *   <li>Exposes {@code applyRequest()} to drive the robot from commands.
 *   <li>Accepts QuestNav vision measurements via {@link #addVisionMeasurement}.
 *   <li>Publishes 2-D and 3-D robot pose to NetworkTables for AdvantageScope.
 *   <li>Publishes individual swerve module states for the AdvantageScope swerve widget.
 *   <li>Runs simulation at 4 ms to keep PID gains well-behaved.
 * </ul>
 *
 * <h2>Coordinate system reminder (WPILib / FRC)</h2>
 * <ul>
 *   <li>+X = robot forward (from driver's perspective)
 *   <li>+Y = robot left
 *   <li>+θ = counter-clockwise (CCW) rotation
 *   <li>Angles are in radians
 * </ul>
 */
public class DrivetrainSubsystem
    extends SwerveDrivetrain<
        com.ctre.phoenix6.hardware.TalonFX,
        com.ctre.phoenix6.hardware.TalonFX,
        com.ctre.phoenix6.hardware.CANcoder>
    implements Subsystem {

  // ─── Simulation ────────────────────────────────────────────────────────────

  /** Simulation loop period – running faster than robot loop improves PID response in sim. */
  private static final double kSimLoopPeriod = 0.004; // 4 ms

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // ─── AdvantageScope / NetworkTables telemetry ──────────────────────────────

  /**
   * Publishes the 2-D robot pose so AdvantageScope can overlay it on a field image.
   *
   * <p>In AdvantageScope: add a "Field2d" tab, set the source to
   * {@code /AdvantageScope/RobotPose2d}.
   */
  private final DoubleArrayPublisher m_pose2dPublisher;

  /**
   * Publishes the 3-D robot pose so AdvantageScope can show a 3-D model of the robot on the field.
   *
   * <p>In AdvantageScope: add a "3D Field" tab, set the source to
   * {@code /AdvantageScope/RobotPose3d}.
   */
  private final DoubleArrayPublisher m_pose3dPublisher;

  /**
   * Publishes the current state (speed + angle) of every swerve module so AdvantageScope's swerve
   * widget can show their orientations.
   *
   * <p>In AdvantageScope: add a "Swerve" tab, set state source to
   * {@code /AdvantageScope/SwerveStates/Measured}.
   */
  private final DoubleArrayPublisher m_moduleStatesPublisher;

  /**
   * Publishes the desired (target) state of every swerve module.
   *
   * <p>In AdvantageScope: set the target source to
   * {@code /AdvantageScope/SwerveStates/Setpoints}.
   */
  private final DoubleArrayPublisher m_moduleSetpointsPublisher;

  // ─── Constructors ──────────────────────────────────────────────────────────

  /**
   * Creates the drivetrain with tuner-generated constants. Called from
   * {@link frc.robot.generated.TunerConstants#createDrivetrain()}.
   *
   * @param drivetrainConstants Drivetrain-wide constants (CAN bus, Pigeon ID, etc.)
   * @param odometryUpdateFrequency Hz for the background odometry thread
   * @param odometryStdDevs Wheel-odometry trust level [X, Y, θ]
   * @param visionStdDevs Vision-measurement trust level [X, Y, θ]
   * @param modules Per-module constants (four modules for a standard swerve)
   */
  @SafeVarargs
  public DrivetrainSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStdDevs,
      Matrix<N3, N1> visionStdDevs,
      SwerveModuleConstants<?, ?, ?>... modules) {

    super(
        com.ctre.phoenix6.hardware.TalonFX::new,
        com.ctre.phoenix6.hardware.TalonFX::new,
        com.ctre.phoenix6.hardware.CANcoder::new,
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStdDevs,
        visionStdDevs,
        modules);

    // Register this as a WPILib subsystem (enables periodic() calls, command requirements)
    registerTelemetry(this::updateTelemetry);

    // ── NetworkTables publishers ──────────────────────────────────────────────
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table = nt.getTable("AdvantageScope");

    m_pose2dPublisher      = table.getDoubleArrayTopic("RobotPose2d")
        .publish(PubSubOption.periodic(0.02));
    m_pose3dPublisher      = table.getDoubleArrayTopic("RobotPose3d")
        .publish(PubSubOption.periodic(0.02));
    m_moduleStatesPublisher = table.getSubTable("SwerveStates")
        .getDoubleArrayTopic("Measured")
        .publish(PubSubOption.periodic(0.02));
    m_moduleSetpointsPublisher = table.getSubTable("SwerveStates")
        .getDoubleArrayTopic("Setpoints")
        .publish(PubSubOption.periodic(0.02));

    // Start the fast simulation notifier if in simulation
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      startSimThread();
    }
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Returns a {@link Command} that, while scheduled, continuously applies the given
   * {@link SwerveRequest} supplier to the drivetrain.
   *
   * <p>Usage example in RobotContainer (field-centric drive):
   * <pre>{@code
   * var fieldCentric = new SwerveRequest.FieldCentric();
   * drivetrain.applyRequest(() ->
   *     fieldCentric
   *         .withVelocityX(translationX)
   *         .withVelocityY(translationY)
   *         .withRotationalRate(rotation)
   * );
   * }</pre>
   *
   * @param requestSupplier Lambda that returns the desired swerve request each loop
   * @return A command that applies the request every loop until cancelled
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Returns the current estimated robot pose as a {@link Pose2d}.
   *
   * @return Robot pose (X meters, Y meters, heading radians)
   */
  public Pose2d getPose2d() {
    return getState().Pose;
  }

  /**
   * Returns the current estimated robot pose as a {@link Pose3d}.
   *
   * <p>The Z component and pitch/roll are zero because the robot is assumed to drive on a flat
   * field surface. These could be populated from QuestNav if desired in the future.
   *
   * @return Robot pose (X m, Y m, Z=0, roll=0, pitch=0, yaw=heading)
   */
  public Pose3d getPose3d() {
    Pose2d pose2d = getPose2d();
    return new Pose3d(
        pose2d.getX(),
        pose2d.getY(),
        0.0, // Field surface; update if needed
        new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()));
  }

  /**
   * Returns the current robot-relative chassis speeds.
   *
   * <p>Required by PathPlanner's AutoBuilder for path following.
   *
   * @return Robot-relative ChassisSpeeds (vx forward, vy left, omega CCW)
   */
  public edu.wpi.first.math.kinematics.ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  /**
   * Seeds the pose estimator with a known pose.
   *
   * <p>Call at the start of autonomous or whenever the robot's true field-relative position is
   * known (e.g. from AprilTags or a fixed starting location).
   *
   * @param pose The pose to seed
   */
  public void seedPose(Pose2d pose) {
    resetPose(pose);
  }

  /**
   * Sets the stator current limit on all four drive motors.
   *
   * @param amps Stator current limit in amps
   */
  public void setDriveCurrentLimit(double amps) {
    var config = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(amps)
        .withStatorCurrentLimitEnable(true);
    for (int i = 0; i < 4; i++) {
      getModule(i).getDriveMotor().getConfigurator().apply(config);
    }
  }

  // ─── Periodic ──────────────────────────────────────────────────────────────

  /**
   * Called every robot loop by the CommandScheduler. Publishing telemetry is handled in
   * {@link #updateTelemetry(SwerveDriveState)} which is invoked synchronously from the odometry
   * thread, so this method is intentionally minimal.
   */
  @Override
  public void periodic() {
    // Nothing extra needed here; all heavy lifting is in registerTelemetry callback.
  }

  // ─── Telemetry Callback ────────────────────────────────────────────────────

  /**
   * Called by CTRE's odometry thread every time new state data is available (up to 250 Hz). Pushes
   * the current pose and module states to NetworkTables for AdvantageScope.
   *
   * <p>This runs on the odometry thread, not the main robot thread, so all operations must be
   * thread-safe (primitive writes and NT4 publishers are safe).
   *
   * @param state The latest swerve drive state from the CTRE stack
   */
  private void updateTelemetry(SwerveDriveState state) {
    Pose2d pose2d = state.Pose;

    // ── 2-D pose ─────────────────────────────────────────────────────────────
    // Format: [x_m, y_m, heading_rad] – what AdvantageScope's Field2d expects
    m_pose2dPublisher.set(new double[] {
        pose2d.getX(),
        pose2d.getY(),
        pose2d.getRotation().getRadians()
    });

    // ── 3-D pose ─────────────────────────────────────────────────────────────
    // Format: [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]
    m_pose3dPublisher.set(new double[] {
        pose2d.getX(),
        pose2d.getY(),
        0.0, // robot is on the ground
        0.0, // roll
        0.0, // pitch
        pose2d.getRotation().getRadians()
    });

    // ── Module states ─────────────────────────────────────────────────────────
    // AdvantageScope swerve widget expects alternating [speed_m_s, angle_rad] pairs
    // for each module in the order: FL, FR, BL, BR.
    SwerveModuleState[] measured = state.ModuleStates;
    SwerveModuleState[] targets  = state.ModuleTargets;

    if (measured != null && measured.length == 4) {
      m_moduleStatesPublisher.set(modulesToDoubleArray(measured));
    }
    if (targets != null && targets.length == 4) {
      m_moduleSetpointsPublisher.set(modulesToDoubleArray(targets));
    }
  }

  /**
   * Converts an array of {@link SwerveModuleState} objects into the flat double array format
   * expected by AdvantageScope's swerve widget: [speed0, angle0, speed1, angle1, …].
   */
  private static double[] modulesToDoubleArray(SwerveModuleState[] states) {
    double[] out = new double[states.length * 2];
    for (int i = 0; i < states.length; i++) {
      out[i * 2]     = states[i].speedMetersPerSecond;
      out[i * 2 + 1] = states[i].angle.getRadians();
    }
    return out;
  }

  // ─── Simulation ────────────────────────────────────────────────────────────

  /**
   * Starts a high-frequency Notifier for the simulation loop.
   *
   * <p>Running at 4 ms (250 Hz) instead of the normal 20 ms robot loop makes PID control gains
   * behave much more like real hardware.
   */
  private void startSimThread() {
    m_lastSimTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    m_simNotifier = new Notifier(() -> {
      double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}