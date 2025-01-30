package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

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
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();

    //ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(5, 10));
    HolonomicDriveController controller = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
    double[] pose;
    double d;
    ChassisSpeeds vel;
    double poseX,poseY;
    Pigeon2 pigeon = new Pigeon2(45);
    Timer timer = new Timer();
    Rotation2d rot,rot2;
    Pose2d desPos,curPos;
    Translation2d mid;
    TrajectoryConfig config;
    Trajectory trajectory;
    boolean trajectoryGenerated = false;

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
    }

    public void update() {
        swerve.swerve(joystick);
        if(joystick.a().getAsBoolean()) {
            if(!trajectoryGenerated) {
                generateTrajectory();
                timer.start();
                trajectoryGenerated = true;
            }
            followTrajectory();
            if(trajectory.getTotalTimeSeconds() >= timer.get()) {
                timer.restart();
                trajectoryGenerated = false;
            }
        }
    }

    public void generateTrajectory() {
        rot = new Rotation2d(pigeon.getYaw(true).getValueAsDouble());
        rot2 = new Rotation2d();
        desPos = new Pose2d(-0.25, 0, rot);
        curPos = new Pose2d(poseX, poseY, rot2);
        mid = new Translation2d((poseX / 2), (poseY / 2));
        config = new TrajectoryConfig(3, 2);
        trajectory = TrajectoryGenerator.generateTrajectory(curPos, List.of(mid), desPos, config);
        System.out.println("Trajectory: " + trajectory.sample(timer.get()));
    }

    public void followTrajectory() {
        //System.out.println("PoseX: " + poseX + "\tPoseY: " + poseY);
        vel = controller.calculate(curPos, trajectory.sample(timer.get()), rot2);
        //System.out.println("Velocity: " + vel);
        swerve.adjust(vel.vxMetersPerSecond, vel.vyMetersPerSecond, vel.omegaRadiansPerSecond);
    }

    public void readPeriodicInputs() {
        pose = limelight.getPose();
        poseX = -pose[0];
        poseY = -pose[2];
        d = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
