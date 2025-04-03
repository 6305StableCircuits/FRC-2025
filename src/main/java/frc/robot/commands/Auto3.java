package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class Auto3 extends Command {

    Timer autoTimer;
    Drive drive = Drive.getInstance();

    public Auto3() {
        autoTimer = new Timer();
    }

    public void initialize() {
        autoTimer.start();
    }

    public void execute() {
        if(autoTimer.get() > 0.750) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(0)).schedule();
        } else {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(1)).schedule();
        }
    }
}
