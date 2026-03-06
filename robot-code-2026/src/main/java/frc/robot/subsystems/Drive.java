// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/**
 * Drive – thin SubsystemBase wrapper around {@link DrivetrainSubsystem} for command compatibility.
 *
 * <p>DrivetrainSubsystem implements the WPILib {@code Subsystem} interface directly (not via
 * SubsystemBase), which means it won't auto-register with the CommandScheduler. Drive extends
 * SubsystemBase so that commands can declare it as a requirement in the normal WPILib way.
 *
 * <p>All meaningful drive logic lives in {@link DrivetrainSubsystem}; this class just delegates.
 */
public class Drive extends SubsystemBase {

  private final DrivetrainSubsystem m_drivetrain;

  public Drive(DrivetrainSubsystem drivetrain) {
    m_drivetrain = drivetrain;
  }

  /**
   * Returns a command that, while scheduled, continuously applies the given swerve request.
   *
   * <p>The returned command requires {@code this} (Drive), not DrivetrainSubsystem, so it
   * plays nicely with the CommandScheduler.
   *
   * @param requestSupplier lambda returning the swerve request each loop
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> m_drivetrain.setControl(requestSupplier.get()));
  }

  /** Seeds the drivetrain pose estimator with a known pose (e.g. from QuestNav). */
  public void seedPose(Pose2d pose) {
    m_drivetrain.seedPose(pose);
  }

  /** Seeds field-centric heading (makes current robot direction = "forward"). */
  public void seedFieldRelative() {
    m_drivetrain.seedFieldCentric();
  }

  /** Returns the latest estimated robot pose. */
  public Pose2d getPose2d() {
    return m_drivetrain.getPose2d();
  }

  /** Returns the underlying DrivetrainSubsystem for subsystems that need direct access. */
  public DrivetrainSubsystem getDrivetrain() {
    return m_drivetrain;
  }
}
