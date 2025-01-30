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
    double kP = 0.065;
    double kI = 0.05;
    double kD = 0;
    double deadband = 1;
    double prevError = 0;
    double errorSum = 0;
    double[] pose;
    double d;
    ChassisSpeeds vel;
    double poseX,poseY;
    int q = 0;
    Pigeon2 pigeon = new Pigeon2(45);
    Timer timer = new Timer();
    Rotation2d rot,rot2;
    Pose2d desPos,curPos;
    Translation2d mid;
    TrajectoryConfig config;
    Trajectory trajectory;

    public static Controls instance = null;
    public static Controls getInstance() {
        if(instance == null) {
            instance = new Controls();
        }
        return instance;
    }

    public Controls() {
        timer.reset();
        q = 0;
    }

    public void update() {
        swerve.swerve(joystick);
        // if(joystick.rightBumper().getAsBoolean()) {
        //     if(limelight.getLock()) {
        //         // if((Math.abs(limelight.getXOffset())) != 0) {
        //         //     errorSum += (limelight.getXOffset() * 0.02);
        //         //     double output = (limelight.getXOffset() * kP) + (((limelight.getXOffset() - prevError) / 0.02) * kD) + (errorSum * kI);
        //         //     System.out.println(output);
        //         //     double velocity = Math.max(Math.min(output, 4.33), -4.33);
        //         //     swerve.adjust(-velocity);
        //         // } else {
        //         //     errorSum = 0;
        //         // }
        //         // prevError = limelight.getXOffset();
        //         // vel = controller.calculate(0, d);
        //         // double velx = vel * (poseX / d);
        //         // double vely = vel * ((poseY - 0.2) / d);
        //         // swerve.adjust(-velx, -vely);
        //     } // else {
        //     //     errorSum = 0;
        //     // }
        // }
        if(joystick.a().getAsBoolean()) {
            while(q++ == 0) {
                generateTrajectory();
                timer.start();
            }
            System.out.println(q);
            followTrajectory();
            if(trajectory.getTotalTimeSeconds() >= timer.get()) {
                timer.restart();
                q = 0;
            }
        }
    }

    public void generateTrajectory() {
        rot = new Rotation2d(pigeon.getYaw(true).getValueAsDouble());
        rot2 = new Rotation2d();
        desPos = new Pose2d(0, -0.2, rot);
        curPos = new Pose2d(poseX, poseY, rot2);
        mid = new Translation2d((poseX / 2), ((poseY - 0.2) / 2));
        config = new TrajectoryConfig(4, 2);
        trajectory = TrajectoryGenerator.generateTrajectory(curPos, List.of(mid), desPos, config);
        System.out.println("Trajectory: " + trajectory.sample(timer.get()));
    }

    public void followTrajectory() {
        System.out.println("PoseX: " + poseX + "\tPoseY: " + poseY);
        vel = controller.calculate(curPos, trajectory.sample(timer.get()), rot2);
        System.out.println("Velocity: " + vel);
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
