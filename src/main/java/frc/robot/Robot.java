// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Limelight;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  SubsystemManager subsystemManager;
  Limelight limelight;
  LEDs leds;
  Drive drive;
  Controls controls;
  
  public Robot() {
    // Instantiate all Subsystems
    limelight = Limelight.getInstance();
    leds = LEDs.getInstance();
    drive = Drive.getInstance();
    controls = Controls.getInstance();

    m_robotContainer = new RobotContainer();

    // Add all Subsystems to the Subsystem Manager
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
      limelight,
      leds,
      drive,
      controls
    ));
  }

  // Update all subsystems on the robot's loop via the Subsystem Manager
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    subsystemManager.updateSubsystems();
    subsystemManager.readSystemsPeriodicInputs();
    subsystemManager.writeSubsystemsPeriodicOutputs();
    subsystemManager.outputSystemsTelemetry(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
