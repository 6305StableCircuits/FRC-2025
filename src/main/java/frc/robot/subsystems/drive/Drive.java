package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;

public class Drive extends Subsystem {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = /*1.5 * Math.PI;*/ RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /** Rotational and regular deadband */
    private final double deadband = 0.05;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    @SuppressWarnings("unused")
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public static Drive instance = null;
    public static Drive getInstance() {
        if(instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public void swerve(CommandXboxController joystick) {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }

    public void resetHeading(CommandXboxController joystick) {}

    public void adjust(double velocityX, double velocityY, double velocityOmega) {
        drivetrain.setControl(drive.withVelocityX(velocityX));
        drivetrain.setControl(drive.withVelocityY(velocityY));
        drivetrain.setControl(drive.withRotationalRate(velocityOmega));
    }

    public void sysIDFwdDynamic() {
        drivetrain.sysIdDynamic(Direction.kForward);
    }

    public void sysIDBwdDynamic() {
        drivetrain.sysIdDynamic(Direction.kReverse);
    }

    public void sysIDFwdQuasistatic() {
        drivetrain.sysIdQuasistatic(Direction.kForward);
    }

    public void sysIDBwdQuasistatic() {
        drivetrain.sysIdQuasistatic(Direction.kReverse);
    }

    public Command followPathCommand(PathPlannerPath path) {
        try{
            return new FollowPathCommand(
                    path,
                    () -> drivetrain.getState().Pose, // Robot pose supplier
                    () -> drivetrain.getKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (ChassisSpeeds speeds, DriveFeedforwards ff) -> drivetrain.setControl(drive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    Constants.robotConfig, // The robot configuration
                    () -> {
                      // Boolean supplier that controls when the path will be mirrored for the red alliance
                      // This will flip the path being followed to the red side of the field.
                      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                      var alliance = DriverStation.getAlliance();
                      if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                      }
                      return false;
                    },
                    drivetrain // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }

    @Override
    public void outputTelemetry() {
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    public void stop() {}
}
