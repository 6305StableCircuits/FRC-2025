// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightLeft;
import frc.robot.subsystems.vision.LimelightRight;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  SubsystemManager subsystemManager;
  LimelightRight limelightRight;
  LimelightLeft limelightLeft;
  LEDs leds;
  Drive drive;
  Controls controls;
  Elevator elevator;
  Shooter shooter;
  
  public Robot() {
    // Instantiate all Subsystems
    controls = Controls.getInstance();
    elevator = Elevator.getInstance();
    shooter = Shooter.getInstance();
    limelightRight = LimelightRight.getInstance();
    limelightLeft = LimelightLeft.getInstance();
    leds = LEDs.getInstance();
    drive = Drive.getInstance();

    m_robotContainer = new RobotContainer();

    // Add all Subsystems to the Subsystem Manager
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
      limelightRight,
      limelightLeft,
      leds,
      drive,
      controls,
      elevator,
      shooter
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
    controls.setTELEOP();
    // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //   drive.drivetrain.getPigeon2().setYaw(drive.drivetrain.getPigeon2().getYaw().getValueAsDouble() + 180);
    // }
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
