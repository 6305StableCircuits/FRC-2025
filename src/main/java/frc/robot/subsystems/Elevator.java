package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class Elevator extends Subsystem {
    public TalonFX hunter;
    public TalonFX garrett;
    public TalonFXConfiguration hunterConfig;
    public TalonFXConfiguration garrettConfig;
    public ProfiledPIDController hunterController;
    public ProfiledPIDController garrettController;
    
    public static Elevator instance = null;
    public static Elevator getInstance() {
        if(instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public Elevator() {
        hunterConfig = new TalonFXConfiguration();
        garrettConfig = new TalonFXConfiguration();
        hunterController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(2, 4));
        garrettController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(2, 4));
        hunter = new TalonFX(Constants.hunterID);
        garrett = new TalonFX(Constants.garrettID);
    }

    public void elevatorUp() {
        hunter.set(0.2);
        garrett.set(-0.2);
    }

    public void elevatorDown() {
        hunter.set(-0.2);
        garrett.set(0.2);   
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
    
}
