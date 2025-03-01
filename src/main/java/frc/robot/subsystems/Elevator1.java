package frc.robot.subsystems


import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.Real.*;

import com.ctre.phoenix6.configs.Slot0configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

	private TalonFX hunter;
	private TalonFXConfigurator hunterConfig;
	private TalonFXConfiguration hunterConfiguration;

	private TalonFX garrett;
	private TalonFXConfigurator garrettConfigurator;
	private TalonFXConfiguration garrettConfigurations;

	private ProfiledPIDController pidController;
	private ElevatorFeedforward ffcontroller;

	private double hunterMotorVoltage;
	private double garrettMotorVoltage;

	private boolean hunterMotorZeroed;

	public ElevatorIOReal() {
		hunter = new TalonFX(Constants.hunterID,"Canviore");
		garrett = new TalonFX(Constants.garrettID,"Canviore");

		hunterVoltage = 0;
		garrettVoltage = 0;

		hunterZero = false;

		//Motor configs
		hunterConfigurator = hunterMotor.getConfigurator();
		garrettConfigurator = garrettMotor.getConfigurator();

		hunterConfigurations = new TalonFXConfiguration();
		garrettConfigurations = new TalonFXConfiguration();

		hunterConfigurations.MotorOutput.Inverted = LEFT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		hunterConfigurations.MotorOutput.NeutralMode = LEFT_NEUTRAL_MODE;
		hunterConfigurations.CurrentLimits.StatorCurrentLimitEnable = LEFT_STRATOR_CURRENT_LIMIT_ENABLED;
		hunterConfigurations.CurrentLimits.StatorCurrentLimit = LEFT_STRATOR_CURRENT_LIMIT.in(Amps);
		hunterConfigurator.apply(hunterConfigurations);

		garrettConfigurations.MotorOutput.Inverted = RIGHT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		garrettConfigurations.MotorOutput.NeutralMode = RIGHT_NEUTRAL_MODE;
		garrettConfigurations.CurrentLimits.StatorCurrentLimitEnable = RIGHT_STRATOR_CURRENT_LIMIT_ENABLED;
		garrettConfigurations.CurrentLimits.StatorCurrentLimit = RIGHT_STRATOR_CURRENT_LIMIT.in(Amps);
		garrettConfigurator.apply(rightConfigurations);

		//PID and FF controller setup
		pidController = new ProfiledPIDController(PROFILLED_PID_CONSTANTS.kP, PROFILLED_PID_CONSTANTS.kI, PROFILLED_PID_CONSTANTS.kD, ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
		pidController.setTolerance(ElevatorConstants.POSITION_TOLERANCE.in(Meters), ElevatorConstants.VELOCITY_TOLERANCE.in(MetersPerSecond));
		pidController.setIZone(PROFILLED_PID_CONSTANTS.iZone);

		ffcontroller = new ElevatorFeedforward(FF_CONSTANTS.kS, FF_CONSTANTS.kG, FF_CONSTANTS.kV, FF_CONSTANTS.kA);



		hunter.setPosition(Degrees.of(0));
		garrett.setPosition(Degrees.of(0));

		garrett.setControl(new Follower(Constants.hunterID, false));\\check
	}

	@Override
	public void setHeightGoalpoint(Distance height) {
		pidController.setGoal(height.in(Meters));
	}

	@Override
	public Distance getHeight() {
		return METERS_PER_ROTATION.times(leftMotor.getPosition().getValueAsDouble());
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs.currentElevatorHeight = hunter.getPosition().getValue().in(Rotations) * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().position;
		inputs.elevatorVelocity = leftMotor.getVelocity().getValue().in(RotationsPerSecond) * METERS_PER_ROTATION.in(Meters);
		inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
		inputs.elevatorHeightGoalpoint = pidController.getGoal().velocity;
		inputs.leftMotorVoltInput = huntertMotorVoltage;
		inputs.rightMotorVoltInput = garrettMotorVoltage;


	}

	@Override
	public void runElevator() {
		leftMotorVoltage = pidController.calculate(leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters)) + ffcontroller.calculate(pidController.getSetpoint().velocity);
		leftMotor.setVoltage(leftMotorVoltage);
	}

	@Override
	public boolean nearTarget() {
		return pidController.atGoal();
	}

	@Override
	public void zero() {
		double leftZeroingSpeed = -ElevatorConstants.ZEROING_VELOCITY.in(MetersPerSecond);
		if (leftMotor.getStatorCurrent().getValueAsDouble() > ElevatorConstants.ZEROING_CURRENT_LIMIT.in(Amps)) {
			leftZeroingSpeed = 0;
			if (!leftMotorZeroed) leftMotor.setPosition(0);
			leftMotorZeroed = true;
		}
		leftMotor.set(leftZeroingSpeed);
	}

	@Override
	public void resetMotorsZeroed() {
		leftMotorZeroed = false;
	}

	@Override
	public boolean motorsZeroed() {
		return leftMotorZeroed;
	}

	@Override
	public Distance getStageOneHeight() {
		return Meters.of(leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters));
	}

	@Override
	public Distance getCarriageHeight() {
		return Meters.of(2 * (leftMotor.getPosition().getValueAsDouble() * METERS_PER_ROTATION.in(Meters)));
	}

	@Override
	public TalonFX getLeftMotor() {
		return leftMotor;
	}

	@Override
	public TalonFX getRightMotor() {
		return rightMotor;
	}
}
