package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {
    private static final double drivekP = 2.0;
    private static final double drivekD = 0.0;
    private static final double thetakP = 5.0;
    private static final double thetakD = 0.0;
    private static final double driveMaxVelocity = Units.inchesToMeters(150.0);
    private static final double driveMaxAcceleration = Units.inchesToMeters(95.0);
    private static final double thetaMaxVelocity = Units.degreesToRadians(360.0);
    private static final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
    private static final double driveTolerance = 0.01;
    private static final double thetaTolerance = Units.degreesToRadians(1.0);
    private static final double ffMinRadius = 0.2;
    private static final double ffMaxRadius = 0.8;
}