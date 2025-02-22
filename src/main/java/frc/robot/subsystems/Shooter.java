package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class Shooter extends Subsystem {
    public SparkMax sasha;
    public SparkMax makena;
    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public Shooter() {
        sasha = new SparkMax(Constants.sashaID, null);
        makena = new SparkMax(Constants.makenaID, null);
    }

    public void reverse() {
        sasha.set(0.2);
        makena.set(-0.2);
    }

    public void forward() {
        sasha.set(-0.2);
        makena.set(0.2);
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