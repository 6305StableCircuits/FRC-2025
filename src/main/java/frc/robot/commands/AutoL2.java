package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class AutoL2 extends Command {
    
    Elevator elevator = Elevator.getInstance();

    public void execute() {
        elevator.raiseL2();
        while(elevator.getRotations() < 19.35) {
            continue;
        }
    }

    public void end() {}
}
