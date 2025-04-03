package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class Auto4 extends Command {
    Elevator elevator = Elevator.getInstance();
    Shooter shooter = Shooter.getInstance();
    Drive drive = Drive.getInstance();
    Timer autoTimer;

    public Auto4() {
        autoTimer = new Timer();
    }

    public void initialize() {
        elevator.blip();
        autoTimer.start();
    }

    public void execute() {
        if(autoTimer.get() >= 11) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withRotationalRate(0)).schedule();
            elevator.resetElevator();
        } else if(autoTimer.get() >= 9) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(0)).schedule();
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withRotationalRate(0.5)).schedule();
        } 
        else if(autoTimer.get() >= 7) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityY(0)).schedule();
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(0.5)).schedule();
        } else if(autoTimer.get() >= 5) {
            shooter.stopShooter();
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityY(-0.50)).schedule();
        } else if(autoTimer.get() >= 3.5) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(0)).schedule();
            shooter.sasha.set(0.4);
            shooter.makena.set(-0.3);
        } else if(autoTimer.get() >= 2.000) {
            drive.drivetrain.applyRequest(() -> drive.swerveroni2.withVelocityX(1)).schedule();
        }
    }

    public void end() {
        autoTimer.reset();
    }
}
