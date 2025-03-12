package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

    //final TrapezoidProfile hunterProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.0625));
    final MotionMagicVoltage hunterRequest = new MotionMagicVoltage(0);
    //final PositionVoltage hunterRequest = new PositionVoltage(0).withSlot(0);
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
        // garrettConfig = new TalonFXConfiguration();
        hunter = new TalonFX(Constants.hunterID, "Canviore");
        garrett = new TalonFX(Constants.garrettID, "Canviore");
        hunter.setPosition(0);
        // garrett.setPosition(0);
        hunterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        hunter.setNeutralMode(NeutralModeValue.Brake);
        garrett.setNeutralMode(NeutralModeValue.Brake);
        MotionMagicConfigs mm = hunterConfig.MotionMagic; // 80, 80, 90
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(80))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80)).withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(90));
        Slot0Configs slot0 = hunterConfig.Slot0;
        slot0.kP = 21; // was 21
        slot0.kI = 0.5; // was 0.5
        slot0.kG = 0.77; // was 0.75
        slot0.kV = 1.46; // was 1.46
        slot0.kA = 0.5; // was 0.5

        limitSwitch = new DigitalInput(0);
        hunter.getConfigurator().apply(hunterConfig);
    }

    public double heightToMotorDegrees(double heightRequest) {
        return 0.225 * heightRequest;
    }

    public double getRotations() {
        return hunter.getPosition().getValueAsDouble();
    }

    public void resetElevator() {
        hunter.setControl(hunterRequest.withPosition(0).withSlot(0));
        garrett.setControl(garrettRequest);
    }

    public void raiseL2() {
        hunter.setControl(hunterRequest.withPosition(15.5).withSlot(0)); // 15.5
        garrett.setControl(garrettRequest);
    }

    public void raiseL3() {
        hunter.setControl(hunterRequest.withPosition(28.25).withSlot(0)); // 28.25
        garrett.setControl(garrettRequest);
    }

    public void rasieL4() {
        hunter.setControl(hunterRequest.withPosition(34).withSlot(0));
        garrett.setControl(garrettRequest);
    }

    @Override
    public void outputTelemetry() {
        //System.out.println(hunter.getPosition().getValueAsDouble());
        
        //System.out.println(hunter.getAcceleration().getValueAsDouble());
    }

    @Override
    public void stop() {}
    
}
