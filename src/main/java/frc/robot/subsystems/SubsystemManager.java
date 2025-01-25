package frc.robot.subsystems;

import java.util.List;

public class SubsystemManager {

    private List<Subsystem> allSubsystems; // list of all subsystems in the codebase to be used for handling updates, outputs, etc. for all of them at once

    public SubsystemManager() {}

    // Mutator method to append subsystems to the list
    public void addSystems(List<Subsystem> subsystemList) {
        allSubsystems = subsystemList;
    }

    // A set of methods below runs a series of respective methods in each Subsystem on a loop via the robot's 
    // periodic loop in Robot.java

    public void readSystemsPeriodicInputs() {
        allSubsystems.forEach((system)-> {system.readPeriodicInputs();});
    }
    public void writeSubsystemsPeriodicOutputs() {
        allSubsystems.forEach((system)->{system.writePeriodicOutputs();});
    }
    public void updateSubsystems() {
        allSubsystems.forEach((system)-> {system.update();});
    }
    public void outputSystemsTelemetry() {
        allSubsystems.forEach((system)-> {system.outputTelemetry();});
    }
    public void stopSubsystems() {
        allSubsystems.forEach((system)-> {system.stop();});
    }
}