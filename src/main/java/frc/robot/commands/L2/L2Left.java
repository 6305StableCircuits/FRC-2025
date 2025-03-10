// package frc.robot.commands.L2;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.drive.Controls;

// public class L2Left extends Command {
//     public Elevator elevator = Elevator.getInstance();
//     public Shooter shooter = Shooter.getInstance();
//     public double prevVel = 0;
//     public Controls align;

//     public L2Left(Controls controls) {
//         addRequirements(elevator, shooter, controls);
//         align = controls;
//     }

//     public void execute() {
//         while(align.getXError() < 0.005 || align.getYError() < .03) {
//             align.alignLeft();
//             if(align.getYVelocity() < prevVel) {
//                 elevator.raiseL2();
//             }
//             prevVel = align.getYVelocity();
//         }
//         shooter.forward();
//         try {
//             wait(350);
//         } catch (InterruptedException e) {
//             // TODO Auto-generated catch block
//             e.printStackTrace();
//         }
//         shooter.stopShooter();
//         elevator.resetElevator();
//     }
// }
