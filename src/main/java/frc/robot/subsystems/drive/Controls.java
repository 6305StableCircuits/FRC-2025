package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();

    ProfiledPIDController controller = new ProfiledPIDController(2, 0.02, 0, new TrapezoidProfile.Constraints(5, 10));
    double kP = 0.065;
    double kI = 0.05;
    double kD = 0;
    double deadband = 1;
    double prevError = 0;
    double errorSum = 0;
    double[] pose;
    double d;
    double vel;

    public static Controls instance = null;
    public static Controls getInstance() {
        if(instance == null) {
            instance = new Controls();
        }
        return instance;
    }

    public void update() {
        swerve.swerve(joystick);
        if(joystick.rightBumper().getAsBoolean()) {
            if(limelight.getLock()) {
                // if((Math.abs(limelight.getXOffset())) != 0) {
                //     errorSum += (limelight.getXOffset() * 0.02);
                //     double output = (limelight.getXOffset() * kP) + (((limelight.getXOffset() - prevError) / 0.02) * kD) + (errorSum * kI);
                //     System.out.println(output);
                //     double velocity = Math.max(Math.min(output, 4.33), -4.33);
                //     swerve.adjust(-velocity);
                // } else {
                //     errorSum = 0;
                // }
                // prevError = limelight.getXOffset();
                vel = controller.calculate(0, d);
                swerve.adjust(-(vel * Math.cos(limelight.getXOffset())), -(vel * Math.sin(limelight.getXOffset())));
            } // else {
            //     errorSum = 0;
            // }
        }
    }

    public void readPeriodicInputs() {
        pose = limelight.getPose();
        d = Math.sqrt(Math.pow(pose[0], 2) + Math.pow(pose[3], 2));
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
