package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator extends Subsystem {
    public TalonFX hunter;
    public TalonFX garrett;
    public TalonFXConfiguration hunterConfig;
    public TalonFXConfiguration garrettConfig;
    public ProfiledPIDController hunterController;
    public ProfiledPIDController garrettController;
    public ElevatorFeedforward hunterFeedForward;
    public ElevatorFeedforward garrettFeedForward;
    public DigitalInput limitSwitch;
    
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
        hunterController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(2, 4), 0.02);
        garrettController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(2, 4), 0.02);
        hunterFeedForward = new ElevatorFeedforward(0, 0.05, 1.5);
        garrettFeedForward = new ElevatorFeedforward(0, 0.05,1.5);
        hunter = new TalonFX(Constants.hunterID, "Canviore");
        garrett = new TalonFX(Constants.garrettID, "Canviore");
        limitSwitch = new DigitalInput(0);
    }

    public double heightToMotorDegrees(double heightRequest) {
        return 5 * heightRequest;
    }

    public void raiseL2() {
        hunterController.setGoal(heightToMotorDegrees(-10));
        garrettController.setGoal(heightToMotorDegrees(10));
        hunter.setControl(new VoltageOut(hunterController.calculate((hunter.getPosition().getValueAsDouble() * 360)) + hunterFeedForward.calculate(hunterController.getSetpoint().velocity)));
        garrett.setControl(new VoltageOut(garrettController.calculate((garrett.getPosition().getValueAsDouble() * 360)) + garrettFeedForward.calculate(garrettController.getSetpoint().velocity)));
    }

    public void elevatorUp() {
        hunter.set(-0.05);
        garrett.set(0.05);
    }

    public void elevatorDown() {
        if(limitSwitch.get()) {
            hunter.set(0.05);
            garrett.set(-0.05);   
        }
    }

    public void elevatorStop() {
        hunter.set(0);
        garrett.set(0);
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
    
}
