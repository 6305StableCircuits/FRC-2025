package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class AutoElevatorDown extends Command {

    Elevator elevator = Elevator.getInstance();

    public void execute() {
        elevator.resetElevator();
        while(elevator.getRotations() > 0.15) {
            continue;
        }
    }

    public void end() {}
}
