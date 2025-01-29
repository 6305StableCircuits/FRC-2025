package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();

    //ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(5, 10));
    HolonomicDriveController controller = new HolonomicDriveController(new PIDController(0.05, 0, 0), new PIDController(0.05, 0, 0), new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
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
    int i = 0;
    Pigeon2 pigeon = new Pigeon2(45);

    public static Controls instance = null;
    public static Controls getInstance() {
        if(instance == null) {
            instance = new Controls();
        }
        return instance;
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
            double t = 0;
            Rotation2d rot = new Rotation2d(pigeon.getYaw(true).getValueAsDouble());
            Rotation2d rot2 = new Rotation2d();
            Pose2d pos = new Pose2d(0, 0, rot);
            Pose2d pos2 = new Pose2d(poseX, (poseY - 0.2), rot2);
            TrajectoryConfig config = new TrajectoryConfig(0, 0);
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(pos, null, pos2, config);

            vel = controller.calculate(pos, trajectory.sample(t), rot2);
            t += 0.2;
            swerve.adjust(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
        }
    }

    public void readPeriodicInputs() {
        pose = limelight.getPose();
        poseX = pose[0];
        poseY = pose[3];
        d = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
