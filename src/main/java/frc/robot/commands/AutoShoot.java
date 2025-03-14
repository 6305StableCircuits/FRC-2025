package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {

    Shooter shooter = Shooter.getInstance();
    
    @Override
    public void execute() {
        shooter.forward();
        try {
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        shooter.stopShooter();
    }
    
    public void end() {}
}
