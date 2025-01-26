package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();

    double kP = 0.1;
    double kD = 0.05;
    double deadband = 1;
    double prevError = 0;

    public static Controls instance = null;
    public static Controls getInstance() {
        if(instance == null) {
            instance = new Controls();
        }
        return instance;
    }

    public void update() {
        swerve.swerve(joystick);

        if(limelight.getLock()) {
            if((Math.abs(limelight.getXOffset())) > deadband) {
                double output = (limelight.getXOffset() * kP) + ((limelight.getXOffset() - prevError) * kD);
                double velocity = Math.max(Math.min(output, 4.33), -4.33);
                swerve.adjust(velocity);
            }
            prevError = limelight.getXOffset();
        }
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
