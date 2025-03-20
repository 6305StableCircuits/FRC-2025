package frc.robot.subsystems.drive;

import java.util.List;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.States;
//import frc.robot.commands.L2.L2Left;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.vision.Limelight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    GenericHID buttonBoard = new GenericHID(1);
    Drive swerve = Drive.getInstance();
    Limelight limelight = Limelight.getInstance();
    Elevator elevator = Elevator.getInstance();
    Shooter shooter = Shooter.getInstance();
    //L2Left l2Left = new L2Left(this);

    //ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(5, 10));
    //HolonomicDriveController controller = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
    double[] pose;
    double d;
    ChassisSpeeds vel;
    double poseX,poseY,yaw;
    Pigeon2 pigeon = new Pigeon2(30);
    Timer timer = new Timer();
    Rotation2d rot,rot2;
    Pose2d desPos,curPos;
    Translation2d mid;
    TrajectoryConfig config;
    Trajectory trajectory;
    boolean trajectoryGenerated = false;
    PathPlannerPath path;
    //kI for both was 0.025
    ProfiledPIDController xController = new ProfiledPIDController(0.75, 0.017, 0, new TrapezoidProfile.Constraints(3, 1));
    ProfiledPIDController yController = new ProfiledPIDController(0.75, 0.017, 0, new TrapezoidProfile.Constraints(3, 1));
    ProfiledPIDController rotController = new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(3, 1));

    ChassisSpeeds appliedSpeed = new ChassisSpeeds();

    double prevVel = 0;
    double velX = 0;
    double velY = 0;
    double velOmega = 0;

    DigitalInput beamBreak = new DigitalInput(0);

    LEDs leds = LEDs.getInstance();

    boolean intake = false;
    double prevReadout = 0;
    boolean runningIntake = false;
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
        xController.setTolerance(.02);
        yController.setTolerance(.08);
        rotController.setTolerance(2);
    }

    public void update() {
        swerve.swerve(joystick);
        // if(buttonBoard.getRawButton(1) && buttonBoard.getRawButton(8)) {
        //     L3Right();
        // } else if(buttonBoard.getRawButton(1) && buttonBoard.getRawButton(5)) {
        //     L3Left();
        // } else if(buttonBoard.getRawButton(2) && buttonBoard.getRawButton(8)) {
        //     L2Left();
        // } else if(buttonBoard.getRawButton(2) & buttonBoard.getRawButton(5)) {
        //     L2Right();
        if(beamBreak.get() == false) {
            if(!limelight.getLock()) {
                States.setState("coralHeld");
            } else if(limelight.getLock() && poseX >= 0.1651 && poseX <= 0.1651 + 0.015) {
                States.setState("Fire!");
            } else if(limelight.getLock() && poseX <= -0.1905 && poseX >= -0.1905 - 0.015) {
                States.setState("Fire!");
            } else {
                States.setState("tagSeen");
            }
        } else {
            States.setState("canIntake");
        }
        if(States.state == "coralHeld") {
            leds.setLEDColor(0, 0, 255);
        } else if(States.state == "tagSeen") {
            leds.setLEDColor(255, 255, 0);
        } else if(States.state == "Fire!") {
            leds.setLEDColor(0, 255, 0);
        } else {
            leds.setLEDColor(255, 0, 0);
        }
        if(buttonBoard.getRawButton(8) && !(States.state == "Fire!")) {
            L2Right();
        } else if(buttonBoard.getRawButton(5) && !(States.state == "Fire!")) {
            L2Left();
        }
        if(buttonBoard.getRawButton(1)) {
            elevator.raiseL3();
        } else if(buttonBoard.getRawButton(2)) {
            elevator.raiseL2();
        } else if(buttonBoard.getRawButton(3)) {
            elevator.resetElevator();
        }
        if(buttonBoard.getRawButton(6) && States.state == "canIntake") {
            shooter.intake();
        } else if(buttonBoard.getRawButton(9)) {
            shooter.forward();
        } else {
            shooter.stopShooter();
        }
        if(buttonBoard.getRawButton(3)) {
            elevator.resetElevator();
        }
        if(buttonBoard.getRawButton(4)) {
            shooter.down();
        } else if(buttonBoard.getRawButton(7)) {
            shooter.up();
        } else {
            shooter.sabrina.stopMotor();
        }
        if(joystick.start().getAsBoolean()) {
            swerve.drivetrain.seedFieldCentric();
        }
        if(joystick.povUp().getAsBoolean()) {
            swerve.drivetrain.setControl(swerve.swerveroni2.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0));
        } else if(joystick.povRight().getAsBoolean()) { // might be backwards??
            swerve.drivetrain.setControl(swerve.swerveroni2.withVelocityY(-0.1).withVelocityX(0).withRotationalRate(0));
        } else if(joystick.povLeft().getAsBoolean()) { // might also be backwards ??
            swerve.drivetrain.setControl(swerve.swerveroni2.withVelocityY(0.1).withVelocityX(0).withRotationalRate(0));
        } else if(joystick.povDown().getAsBoolean()) {
            swerve.drivetrain.setControl(swerve.swerveroni2.withVelocityX(-0.5).withVelocityY(0).withRotationalRate(0));
        }
        if(joystick.x().getAsBoolean()) {
            swerve.drivetrain.setControl(swerve.swerveroni2.withVelocityY(0.25).withVelocityX(0).withRotationalRate(0));
        } else if(joystick.b().getAsBoolean()) {
            swerve.drivetrain.setControl(swerve.swerveroni2.withVelocityY(-0.25).withVelocityX(0).withRotationalRate(0));
        }
    }

    // public void L3Right() {
    //     if(limelight.getLock()) {
    //         appliedSpeed.vyMetersPerSecond = xController.calculate(poseX, -0.1651);
    //         appliedSpeed.vxMetersPerSecond = yController.calculate(poseY, -0.435);
    //         appliedSpeed.omegaRadiansPerSecond = rotController.calculate(yaw, 0);
    //         swerve.adjust();
    //         if(yController.getSetpoint().velocity < prevVel) {
    //             elevator.raiseL3();
    //         }
    //         prevVel = yController.getSetpoint().velocity;
    //         if(28.1 <= elevator.getRotations() && elevator.getRotations() >= 28.4) {
    //             shooter.quickShoot();
    //             elevator.resetElevator();
    //             prevVel = 0;
    //         }
    //         if(elevator.getRotations() <= 28) {
    //             shooter.stopShooter();
    //         }
    //     }
    // }

    // public void L3Left() {
    //     if(limelight.getLock()) {
    //         appliedSpeed.vyMetersPerSecond = xController.calculate(poseX, 0.1691);
    //         appliedSpeed.vxMetersPerSecond = yController.calculate(poseY, -0.435);
    //         appliedSpeed.omegaRadiansPerSecond = rotController.calculate(yaw, 0);
    //         swerve.adjust(appliedSpeed);
    //         if(yController.getSetpoint().velocity < prevVel) {
    //             elevator.raiseL3();
    //         }
    //         prevVel = yController.getSetpoint().velocity;
    //         if(28.1 <= elevator.getRotations() && elevator.getRotations() >= 28.4) {
    //             shooter.quickShoot();
    //             elevator.resetElevator();
    //             prevVel = 0;
    //         }
    //         if(elevator.getRotations() <= 28) {
    //             shooter.stopShooter();
    //         }
    //     }
    // }

    public void L2Right() {
        // appliedSpeed.vyMetersPerSecond = xController.calculate(poseX, -0.1651);
        // appliedSpeed.vxMetersPerSecond = yController.calculate(poseY, -0.435);
        // appliedSpeed.omegaRadiansPerSecond = rotController.calculate(yaw, 0);
        // swerve.adjust(appliedSpeed);
        
        //velY = xController.calculate(poseX, -0.1651); // -0.1651 | -0.1905
        //velX = yController.calculate(poseY, -1);
        velOmega = rotController.calculate(yaw, 0);
        swerve.adjust(0, 0, velOmega);
        // if(yController.getPositionError() < 0.05) {
        //     elevator.raiseL2();
        // }
        // if(15.1 <= elevator.getRotations() && elevator.getRotations() >= 15.9) {
        //     shooter.quickShoot();
        //     elevator.resetElevator();
        // }
        // if(elevator.getRotations() <= 14) {
        //     shooter.stopShooter();
        // }
    }

    public void L2Left() {
        // appliedSpeed.vyMetersPerSecond = xController.calculate(poseX, 0.1691);
        // appliedSpeed.vxMetersPerSecond = yController.calculate(poseY, -0.435);
        // appliedSpeed.omegaRadiansPerSecond = rotController.calculate(yaw, 0);
        // swerve.adjust(appliedSpeed);
        //velY = xController.calculate(poseX, 0.1651); // 1651 | 1945
        //velX = yController.calculate(poseY, -1); // -0.4604
        velOmega = rotController.calculate(yaw, 0);
        swerve.adjust(0, 0, velOmega);
        // if(yController.getPositionError() < 0.05) {
        //     elevator.raiseL2();
        // }
        // if(15.1 <= elevator.getRotations() && elevator.getRotations() >= 15.9) {
        //     shooter.quickShoot();
        //     elevator.resetElevator();
        // }
        // if(elevator.getRotations() <= 14) {
        //     shooter.stopShooter();
        // }
    }

    public void readPeriodicInputs() {
        pose = limelight.getPose();
        poseX = ((-1) * pose[0]);
        poseY = pose[2];
        yaw = ((-1) * pose[4]);
        d = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
