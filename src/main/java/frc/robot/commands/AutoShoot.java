package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {

    Shooter shooter = Shooter.getInstance();

    public void initialize() {
        shooter.forward();
        new WaitCommand(500);
        shooter.stopShooter();
    }
    
    public void end() {}
}
