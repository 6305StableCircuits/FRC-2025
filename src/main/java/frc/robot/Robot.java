// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.vision.Limelight;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  
  SubsystemManager subsystemManager;
  Limelight limelight;
  LEDs candle;
  
  public Robot() {
    // Instantiate all Subsystems
    limelight = Limelight.getInstance();
    candle = LEDs.getInstance();

    // Add all Subsystems to the Subsystem Manager
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
      limelight,
      candle
    ));
  }

  // Update all subsystems on the robot's loop via the Subsystem Manager
  @Override
  public void robotPeriodic() {
    subsystemManager.updateSubsystems();
    subsystemManager.readSystemsPeriodicInputs();
    subsystemManager.writeSubsystemsPeriodicOutputs();
    subsystemManager.outputSystemsTelemetry();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
