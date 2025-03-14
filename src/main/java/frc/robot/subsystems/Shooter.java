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
    // private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(15, 15);
    // private final ProfiledPIDController sabrinaController = new ProfiledPIDController(3, 0.3, 0.02, constraints);
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
        sasha.set(0.6);
        makena.set(-0.5);
        bool = true;
    }

    public void intake() {
        sasha.set(0.5);
        makena.set(-0.5);
    }

    public void quickShoot() {
        sasha.set(0.8);
        makena.set(-0.7);
    }

    public void reverse() {
        sasha.set(-0.4);
        makena.set(0.4);
  }    

    public void down() {
        if(encoder.getPosition() > -22) {
            sabrina.set(-0.2);
        }
    }

    public void up() {
        if(encoder.getPosition() < -1.5) {
            sabrina.set(0.2);
        }
    }

    public void stopShooter() {
        sasha.stopMotor();
        makena.stopMotor();
        bool = false;
    }

    public void update() {}

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}