// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private record MatchPeriodStatus(String period, double periodTime) {
  }

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    MatchPeriodStatus matchPeriodStatus = getCurrentMatchPeriodAndTime();

    SmartDashboard.putString("Match Period", matchPeriodStatus.period());
    SmartDashboard.putNumber("Period Time", matchPeriodStatus.periodTime());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
      ;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private MatchPeriodStatus getCurrentMatchPeriodAndTime() {
    if (DriverStation.isEStopped()) {
      return new MatchPeriodStatus("E-Stopped", 0.0);
    }

    double rawMatchTime = DriverStation.getMatchTime();
    boolean hasMatchTime = rawMatchTime >= 0.0;
    double matchTime = hasMatchTime ? rawMatchTime : 0.0;

    if (DriverStation.isAutonomous()) {
      return new MatchPeriodStatus(
          DriverStation.isEnabled() ? "AUTO" : "AUTO (Disabled)",
          hasMatchTime ? rawMatchTime : 0.0);
    }

    if (DriverStation.isTeleop()) {
      if (!hasMatchTime) {
        return new MatchPeriodStatus(
            DriverStation.isEnabled() ? "TELEOP" : "TELEOP (Disabled)",
            0.0);
      }
      if (!DriverStation.isEnabled() && matchTime <= 0.0) {
        return new MatchPeriodStatus("TELEOP (Disabled)", matchTime);
      } else if (matchTime > 130.0) {
        return new MatchPeriodStatus("TRANSITION SHIFT", matchTime - 130.0);
      } else if (matchTime > 105.0) {
        return new MatchPeriodStatus("SHIFT 1", matchTime - 105.0);
      } else if (matchTime > 80.0) {
        return new MatchPeriodStatus("SHIFT 2", matchTime - 80.0);
      } else if (matchTime > 55.0) {
        return new MatchPeriodStatus("SHIFT 3", matchTime - 55.0);
      } else if (matchTime > 30.0) {
        return new MatchPeriodStatus("SHIFT 4", matchTime - 30.0);
      } else {
        return new MatchPeriodStatus("END GAME", matchTime);
      }
    }

    if (DriverStation.isTest()) {
      return new MatchPeriodStatus(
          DriverStation.isEnabled() ? "Test" : "Test (Disabled)",
          0.0);
    }

    return new MatchPeriodStatus("Disabled", 0.0);
  }
}
