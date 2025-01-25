package frc.robot.subsystems;

// Abstract Subsystem class to be inherited by each actual subsystem
// Names of methods self-explanatory -- update pushes updates, readPeriodicInputs reads periodic inputs, etc.
// All methods in each subsystem will run on a loop via the Subsystem Manager

public abstract class Subsystem {
    public void readPeriodicInputs() {
    }
    public void writePeriodicOutputs() {
    }
    public void update() {
    }
    public abstract void outputTelemetry();

    public abstract void stop();
}
