package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

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
    public double hunterOffset,garrettOffset;
    Slot0Configs hunterSlot0Configs = new Slot0Configs();
    Slot0Configs garrettSlot0Configs = new Slot0Configs();
    final TrapezoidProfile hunterProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.0625));
    final TrapezoidProfile garrettProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.0625));
    final PositionVoltage hunterRequest = new PositionVoltage(0).withSlot(0);
    //final PositionVoltage garrettRequest = new PositionVoltage(0).withSlot(0);
    final Follower garrettRequest = new Follower(10, true);
    
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
        // hunterController = new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.1), 0.02);
        // garrettController = new ProfiledPIDController(0.2, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.1), 0.02);
        // hunterFeedForward = new ElevatorFeedforward(0, 0.5, 0);
        // garrettFeedForward = new ElevatorFeedforward(0, 0.5,0);
        hunter = new TalonFX(Constants.hunterID, "Canviore");
        garrett = new TalonFX(Constants.garrettID, "Canviore");
        hunterSlot0Configs.kG = 1.5;
        garrettSlot0Configs.kG = 1.5;
        hunterSlot0Configs.kP = 50;
        garrettSlot0Configs.kP = 50;
        hunterSlot0Configs.kS = 1.5;
        garrettSlot0Configs.kS = 1.5;
        hunterSlot0Configs.kI = 10;
        garrettSlot0Configs.kI = 10;
        // hunterSlot0Configs.kD = 0.5;
        // garrettSlot0Configs.kD = 0.5;
        limitSwitch = new DigitalInput(0);
        // hunterOffset = hunter.getPosition().getValueAsDouble();
        // garrettOffset = garrett.getPosition().getValueAsDouble();
        hunter.getConfigurator().apply(hunterSlot0Configs);
    }

    public double heightToMotorDegrees(double heightRequest) {
        return 0.225 * heightRequest;
    }

    public void raiseL2() {
        // hunterController.setGoal(-5);
        // garrettController.setGoal(5);
        // hunter.setControl(new PositionVoltage(hunterController.calculate(((hunter.getPosition().getValueAsDouble() - hunterOffset))) + hunterFeedForward.calculate(hunterController.getSetpoint().velocity)));
        // garrett.setControl(new PositionVoltage(garrettController.calculate(((garrett.getPosition().getValueAsDouble() - garrettOffset))) + garrettFeedForward.calculate(garrettController.getSetpoint().velocity)));
        TrapezoidProfile.State hunterGoal = new TrapezoidProfile.State(30, 0);
        TrapezoidProfile.State hunterSetpoint = new TrapezoidProfile.State();
        //TrapezoidProfile.State garrettGoal = new TrapezoidProfile.State(30, 0);
        //TrapezoidProfile.State garrettSetpoint = new TrapezoidProfile.State();

        hunterSetpoint = hunterProfile.calculate(0.02, hunterSetpoint, hunterGoal);
        //garrettSetpoint = garrettProfile.calculate(0.02, garrettSetpoint, garrettGoal);

        hunterRequest.Position = hunterSetpoint.position;
        hunterRequest.Velocity = hunterSetpoint.velocity;
        // garrettRequest.Position = garrettSetpoint.position;
        // garrettRequest.Velocity = garrettSetpoint.velocity;

        hunter.setControl(hunterRequest);
        garrett.setControl(garrettRequest);
    }

    public void elevatorUp() {
        hunter.set(-0.20);
        garrett.set(0.20);
    }

    public void elevatorDown() {
        if(limitSwitch.get()) {
            hunter.set(0.2);
            garrett.set(-0.2);   
        }
    }

    public void elevatorStop() {
        hunter.set(0);
        garrett.set(0);
    }

    @Override
    public void outputTelemetry() {
        //System.out.println(hunter.getPosition().getValueAsDouble());
    }

    @Override
    public void stop() {}
    
}
