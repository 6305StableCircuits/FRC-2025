package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();
    Elevator elevator = Elevator.getInstance();
    Shooter shooter = Shooter.getInstance();

    //ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(5, 10));
    HolonomicDriveController controller = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
    double[] pose;
    double d;
    ChassisSpeeds vel;
    double poseX,poseY,yaw;
    Pigeon2 pigeon = new Pigeon2(45);
    Timer timer = new Timer();
    Rotation2d rot,rot2;
    Pose2d desPos,curPos;
    Translation2d mid;
    TrajectoryConfig config;
    Trajectory trajectory;
    boolean trajectoryGenerated = false;
    PathPlannerPath path;

    public static Controls instance = null;
    public static Controls getInstance() {
        if(instance == null) {
            instance = new Controls();
        }
        return instance;
    }

    public Controls() {
        timer.restart();
        trajectoryGenerated = false;
        pigeon.reset();
    }

    public void update() {
        swerve.swerve(joystick);
        if(joystick.a().getAsBoolean()) {
            if(!trajectoryGenerated) {
                generateTrajectory();
                trajectoryGenerated = true;
                swerve.followPathCommand(path);
            } else {}
            // if(trajectory.getTotalTimeSeconds() >= timer.get()) {
            //     trajectoryGenerated = false;
            // }
        }
        if(joystick.b().getAsBoolean()) {
            elevator.elevatorUp();
        } else if(joystick.x().getAsBoolean()) {
            elevator.elevatorDown();
        } else {
            elevator.elevatorStop();
        }
        if (joystick.leftTrigger().getAsBoolean()) {
            shooter.forward();
        } else if (joystick.rightTrigger().getAsBoolean()) {
            shooter.reverse();
        } else {
            shooter.stopShooter();
        }
    }

    // public void generateTrajectory() {
    //     rot = new Rotation2d(yaw);
    //     rot2 = new Rotation2d();
    //     desPos = new Pose2d(0, 0.2, rot2);
    //     curPos = new Pose2d(poseX, poseY, rot);
    //     mid = new Translation2d((poseX / 2), (poseY / 2));
    //     config = new TrajectoryConfig(3, 2);
    //     trajectory = TrajectoryGenerator.generateTrajectory(curPos, List.of(mid), desPos, config);
    //     System.out.println("Trajectory: " + trajectory.sample(timer.get()));
    // }

    // public void generateTrajectory() {
    //     rot = new Rotation2d(yaw);
    //     rot2 = new Rotation2d();
    //     desPos = new Pose2d(0, 0.2, rot);
    //     curPos = new Pose2d(poseX, poseY, rot);
    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //         curPos,
    //         desPos
    //     );
    //     PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
    //     path = new PathPlannerPath(
    //         waypoints, constraints, null, new GoalEndState(0.0, rot2));
    // }

    public void generateTrajectory() {
                // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
    }

    // public void followTrajectory() {
    //     Rotation2d newRot = new Rotation2d(yaw);
    //     Pose2d curPos2 = new Pose2d(poseX, poseY, newRot);
    //     System.out.println(trajectory);
    //     vel = controller.calculate(curPos2, trajectory.sample(timer.get()), rot2);
    //     System.out.println(curPos2);
    //     swerve.adjust(vel);
    // }

    public void readPeriodicInputs() {
        pose = limelight.getPose();
        poseX = pose[0];
        poseY = pose[2];
        yaw = pose[4];
        d = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
