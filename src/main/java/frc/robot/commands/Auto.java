package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class Auto extends Command {
    Elevator elevator = Elevator.getInstance();
    Shooter shooter = Shooter.getInstance();
    Drive drive = Drive.getInstance();
    Timer autoTimer;

    public Auto() {
        autoTimer = new Timer();
    }

    public void initialize() {
        elevator.blip();
        autoTimer.start();
        // new WaitCommand(1000);
        // drive.drivetrain.setControl(drive.swerveroni2.withVelocityX(1));
        // System.out.print("AHHHHHHHH");
        // new WaitCommand(2235);
        // drive.drivetrain.setControl(drive.swerveroni2.withVelocityX(0));
        // shooter.forward();
        // new WaitCommand(500);
        // shooter.stopShooter();
        // drive.drivetrain.setControl(drive.swerveroni2.withVelocityY(0.25));
        // new WaitCommand(500);
        // drive.drivetrain.setControl(drive.swerveroni2.withVelocityY(0));
    }

    public void execute() {
        if(autoTimer.get() >= 2.000) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(1)).execute();;
        } else if(autoTimer.get() >= 4.235) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(0)).execute();;
            shooter.forward();
        } else if(autoTimer.get() >= 4.735) {
            shooter.stopShooter();
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityY(0.25)).execute();;
        } else if(autoTimer.get() >= 5.325) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityY(0)).execute();;
        }
    }

    public void end() {
        autoTimer.reset();
    }
}
