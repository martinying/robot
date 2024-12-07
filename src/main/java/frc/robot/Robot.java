// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    this.advKitInit();
    m_robotContainer = new RobotContainer();
  }

  private void advKitInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter(this.makeLogFileDirectoryIfNotExists("/homne/lvuser/logs"))); // Log to a
                                                                                                            // USB stick
                                                                                                            // ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution
      // logging
    } else {
      Logger.addDataReceiver(new NT4Publisher());
      // setUseTiming(false); // Run as fast as possible
      // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
      // AdvantageScope (or prompt the user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
      // "_sim"))); // Save outputs to a new log
    }

    Logger.start();
  }

  private String makeLogFileDirectoryIfNotExists(String pathName) {
    File directory = new File(pathName);
    if (!directory.exists()) {
      directory.mkdir();
    }
    return pathName;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    // m_teleopCommand = m_robotContainer.getTeleopCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
