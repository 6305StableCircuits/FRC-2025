package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class Shooter extends Subsystem {
    public SparkMax sasha;
    public SparkMax makena;
    public SparkMax sabrina;
    private static Shooter instance = null;
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
    }

    public void forward() {
        sasha.set(0.2);
        makena.set(-0.2);
    }

    public void reverse() {
        sasha.set(-0.2);
        makena.set(0.2);
    }
    
    public void right() {
        sabrina.set(0.15);
    }

    public void left() {
        sabrina.set(-0.15);
    }

    public void stopSlide() {
        sabrina.stopMotor();
    }

    public void stopShooter() {
        sasha.stopMotor();
        makena.stopMotor();
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}