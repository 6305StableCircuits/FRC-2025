package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public double x = 0;
    public double y = 0;

    // public void update(){
    //     SmartDashboard.putNumber("Current Amperage:", sasha.getOutputCurrent());
    //     if(/*intaking */){
    //         LinearFilter filter = LinearFilter.movingAverage(5);
    //         x = filter.calculate(sasha.getOutputCurrent());
    //         if(sasha.getOutputCurrent() > 9){
    //             //stop intake
    //         }
    //         if((x - y) < 0 && sasha.getOutputCurrent() < 12 /*&& is intaking */){
    //             //pipe? held
    //         }
    //         y = x;
    //     }
    // }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}