// Copyright (c) 2026 Team 801 Horsepower
// Open Source Software - see LICENSE.md in root directory
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Team 801 Horsepower - 2026 Robot Main Class
 *
 * <p>This is the top-level robot class. The CommandScheduler runs on a 20ms loop driven by
 * TimedRobot. All subsystems and commands are instantiated and wired together in {@link
 * RobotContainer}.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * Called once at robot startup. Initializes the DataLogManager for on-robot logging, suppresses
   * DS warnings from disconnected joysticks in simulation, and constructs the RobotContainer.
   */
  @Override
  public void robotInit() {
    // Start on-robot data logging to a USB stick if present
    DataLogManager.start();

    // Log all NetworkTables values automatically
    DriverStation.startDataLog(DataLogManager.getLog());

    // Suppress joystick warnings during simulation
    DriverStation.silenceJoystickConnectionWarning(true);

    m_robotContainer = new RobotContainer();
  }

  /**
   * Runs the CommandScheduler every robot loop (20 ms). This is the heartbeat of the entire
   * command-based framework – all commands and subsystems tick here.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // ─── Disabled ──────────────────────────────────────────────────────────────

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  // ─── Autonomous ────────────────────────────────────────────────────────────

  @Override
  public void autonomousInit() {
    m_robotContainer.seedAutoStartPose();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // ─── Teleop ────────────────────────────────────────────────────────────────

  @Override
  public void teleopInit() {
    // Cancel any running auto command so teleop can take over
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.cacheAlliance();
    m_robotContainer.configureNormalMode();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  // ─── Test ──────────────────────────────────────────────────────────────────

  @Override
  public void testInit() {
    m_robotContainer.configureTestMode();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    m_robotContainer.configureNormalMode();
  }

  // ─── Simulation ────────────────────────────────────────────────────────────

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}