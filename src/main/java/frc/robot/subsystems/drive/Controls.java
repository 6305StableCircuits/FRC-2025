package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    XboxController joystick = new XboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();

    public static Controls instance = null;
    public static Controls getInstance() {
        if(instance == null) {
            instance = Controls.getInstance();
        }
        return instance;
    }

    public void update() {
        swerve.swerve(joystick);

        if(limelight.getLock()) {
            swerve.drivetrain.applyRequest(() -> swerve.drive.withVelocityX(1.0));
        }
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
