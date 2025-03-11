package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.States;

public class Shooter extends Subsystem {
    public SparkMax sasha;
    public SparkMax makena;
    public SparkMax sabrina;
    public RelativeEncoder encoder;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(15, 15);
    private final ProfiledPIDController sabrinaController = new ProfiledPIDController(3, 0.3, 0.02, constraints);
    private static Shooter instance = null;
    public boolean bool;
    public double x,y;
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public Shooter() {
        sasha = new SparkMax(Constants.sashaID, MotorType.kBrushless);
        makena = new SparkMax(Constants.makenaID, MotorType.kBrushless);
        sabrina = new SparkMax(Constants.sabrinaID, MotorType.kBrushless);
        encoder = sabrina.getEncoder();
        encoder.setPosition(0);
    }

    public void forward() {
        sasha.set(0.3);
        makena.set(-0.3);
        bool = true;
    }

    public void reverse() {
        sasha.set(-0.4);
        makena.set(0.4);
  }    

    public void down() {
        sabrinaController.setGoal(-25);
        sabrina.setVoltage(sabrinaController.calculate(encoder.getPosition()));
    }

    public void up() {
        sabrinaController.setGoal(0);
        sabrina.setVoltage(sabrinaController.calculate(encoder.getPosition()));
    }

    public void stopShooter() {
        sasha.stopMotor();
        makena.stopMotor();
        bool = false;
    }

    public void update(){
        LinearFilter filter = LinearFilter.movingAverage(5);
        Debouncer debouncer = new Debouncer(1, DebounceType.kRising);
        if(bool) {
            x = filter.calculate((sasha.getOutputCurrent() + makena.getOutputCurrent()) / 2);
            if(debouncer.calculate(x > 28)) {
                States.setState("coralHeld");
            }
        }
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}