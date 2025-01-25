package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();

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
            if(limelight.getXOffset() > 1) {
                swerve.adjustLeft();
            } else if(limelight.getXOffset() < -1) {
                swerve.adjustRight();
            } else {
                swerve.adjustStop();
            }
        }
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
