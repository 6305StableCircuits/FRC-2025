package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;

public class Drive extends Subsystem {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = /*1.5 * Math.PI;*/ RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /** Rotational and regular deadband */
    private final double deadband = 0.1;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    @SuppressWarnings("unused")
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.ApplyRobotSpeeds swerveroni = new SwerveRequest.ApplyRobotSpeeds();
    public final SwerveRequest.RobotCentric swerveroni2 = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    RobotConfig config;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public static Drive instance = null;
    public static Drive getInstance() {
        if(instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public Drive() {

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            drivetrain::getPose, // Robot pose supplier
            drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drivetrain.setControl(swerveroni.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.2, 0, 0.0), // Translation PID constants 1.5 0.073 0.0 | 0.2 0.0 0.0
                    new PIDConstants(0.8, 0, 0.0) // Rotation PID constants 5.0 0.1 0.0 | 0.8 0.0 0.0
            ),
            config, // The robot configuration
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == Alliance.Red;
                }
                return false;
            },
            drivetrain // Reference to this subsystem to set requirements
    );
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
    
    // public void adjust(ChassisSpeeds speeds) {
    //     drivetrain.setControl(swerveroni.withSpeeds(speeds));
    // }

    public void adjust(double velX, double velY, double velOmega) {
        drivetrain.setControl(swerveroni2.withVelocityX(velX).withVelocityY(velY).withRotationalRate(velOmega));
        System.out.println("VelX: " + velX + " VelY: " + velY + " VelOmega: " + velOmega);
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

    @Override
    public void outputTelemetry() {
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    public void stop() {}
}
