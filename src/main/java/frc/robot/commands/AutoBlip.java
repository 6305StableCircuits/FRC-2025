package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class AutoBlip extends Command {
    
    Elevator elevator = Elevator.getInstance();

    public void initialize() {
        elevator.blip();
    }

    public void end() {}
}
