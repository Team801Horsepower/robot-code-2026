// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * DriveToPose – drives the robot to a target field-relative pose using three independent
 * profiled PID controllers (X, Y, and rotation).
 *
 * <p>TODO: Replace with PathPlanner if added as a vendordep for smoother autonomous paths.
 */
public class DriveToPose extends Command {

  private static final double kXYP  = 3.0;
  private static final double kXYD  = 0.0;
  private static final double kMaxV = 2.0; // m/s
  private static final double kMaxA = 3.0; // m/s²

  private static final double kThetaP   = 5.0;
  private static final double kThetaD   = 0.0;
  private static final double kMaxOmega = Math.PI; // rad/s
  private static final double kMaxAlpha = 2 * Math.PI; // rad/s²

  private static final double kXYTolerance    = 0.05; // m
  private static final double kThetaTolerance = Math.toRadians(2.0);

  private final Drive m_drive;
  private final Pose2d m_targetPose;

  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

  public DriveToPose(Drive drive, Pose2d targetPose) {
    m_drive      = drive;
    m_targetPose = targetPose;

    var xyConstraints    = new TrapezoidProfile.Constraints(kMaxV, kMaxA);
    var thetaConstraints = new TrapezoidProfile.Constraints(kMaxOmega, kMaxAlpha);

    m_xController     = new ProfiledPIDController(kXYP, 0, kXYD, xyConstraints);
    m_yController     = new ProfiledPIDController(kXYP, 0, kXYD, xyConstraints);
    m_thetaController = new ProfiledPIDController(kThetaP, 0, kThetaD, thetaConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d current = m_drive.getPose2d();
    m_xController.reset(current.getX());
    m_yController.reset(current.getY());
    m_thetaController.reset(current.getRotation().getRadians());

    m_xController.setGoal(m_targetPose.getX());
    m_yController.setGoal(m_targetPose.getY());
    m_thetaController.setGoal(m_targetPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d current = m_drive.getPose2d();

    double vx    = m_xController.calculate(current.getX());
    double vy    = m_yController.calculate(current.getY());
    double omega = m_thetaController.calculate(current.getRotation().getRadians());

    m_drive.getDrivetrain().setControl(
        m_driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omega));
  }

  @Override
  public boolean isFinished() {
    Pose2d current = m_drive.getPose2d();
    double dx = current.getX() - m_targetPose.getX();
    double dy = current.getY() - m_targetPose.getY();
    double dtheta = Math.abs(current.getRotation().minus(m_targetPose.getRotation()).getRadians());
    return Math.hypot(dx, dy) < kXYTolerance && dtheta < kThetaTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.getDrivetrain().setControl(
        m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
