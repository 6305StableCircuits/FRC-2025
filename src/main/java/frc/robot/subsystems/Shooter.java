package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends Subsystem {
    public SparkMax sasha;
    public SparkMax makena;
    public SparkMax sabrina;
    public RelativeEncoder encoder;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.5, 0.25);
    private final ProfiledPIDController sabrinaController = new ProfiledPIDController(0.1, 0, 0, constraints);
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
        encoder = sabrina.getEncoder();
        encoder.setPosition(0);
    }

    public void forward() {
        sasha.set(0.6);
        makena.set(-0.5);
    }

    public void reverse() {
        sasha.set(-0.4);
        makena.set(0.4);
        // intaking = true;
    }
    
    public void right() {
        sabrina.set(0.15);
    }

    public void left() {
        sabrina.set(-0.15);
    }

    public void center() {
        sabrinaController.setGoal(0);
        sabrina.setVoltage(sabrinaController.calculate(encoder.getPosition()));
    }

    public void stopSlide() {
        sabrina.stopMotor();
    }

    public void stopShooter() {
        sasha.stopMotor();
        makena.stopMotor();
        // intaking = false;
    }

    // public double filterData = 0;
    // public double filterData2 = 0;
    // public boolean intaking = false;

    public void update(){
        // SmartDashboard.putNumber("Current Amperage:", sasha.getOutputCurrent());
        // if(intaking){
        //     LinearFilter filter = LinearFilter.movingAverage(5);
        //     filterData = filter.calculate(sasha.getOutputCurrent());
        //     if(sasha.getOutputCurrent() > 9){
        //         stopShooter();
        //     }
        //     // if((filterData - filterData2) < 0 && sasha.getOutputCurrent() < 12 && intaking){
        //     //     //pipe(?) held
        //     // }
        //     filterData2 = filterData;
        }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}