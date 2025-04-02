package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class Auto2 extends Command {
    Elevator elevator = Elevator.getInstance();
    Shooter shooter = Shooter.getInstance();
    Drive drive = Drive.getInstance();
    Timer autoTimer;
    boolean shoot;
    int i;

    public Auto2() {
        autoTimer = new Timer();
        shoot = false;
        i = 0;
    }

    public void initialize() {
        elevator.raiseL2();
        autoTimer.start();
    }

    public void execute() {
        if(autoTimer.get() >= 6) {
            shooter.stopShooter();
        } else if(autoTimer.get() >= 4.3) {
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
