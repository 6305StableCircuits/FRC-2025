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
    Pigeon2 pigeon = new Pigeon2(45);
    Timer timer = new Timer();
    Rotation2d rot,rot2;
    Pose2d desPos,curPos;
    Translation2d mid;
    TrajectoryConfig config;
    Trajectory trajectory;
    boolean trajectoryGenerated = false;
    PathPlannerPath path;
    //kI for both was 0.025
    ProfiledPIDController xController = new ProfiledPIDController(0.96, 0.025, 0, new TrapezoidProfile.Constraints(3, 1));
    ProfiledPIDController yController = new ProfiledPIDController(0.96, 0.025, 0, new TrapezoidProfile.Constraints(3, 1));
    ProfiledPIDController rotController = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(3, 1));
    //ProfiledPIDController rotController = new ProfiledPIDController(0.05, 0.06, 0.005, new TrapezoidProfile.Constraints(3, 1));

    ChassisSpeeds appliedSpeed = new ChassisSpeeds();
    ChassisSpeeds littleIttyBittyBabyAdjust = new ChassisSpeeds();

    AnalogInput sensor = new AnalogInput(0);

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
        xController.setTolerance(.005);
        yController.setTolerance(.03);
        rotController.setTolerance(2);
        littleIttyBittyBabyAdjust.vxMetersPerSecond = 1;
    }

    public void update() {
        swerve.swerve(joystick);
        if(joystick.b().getAsBoolean()) {
            alignRight();
        } else if(joystick.x().getAsBoolean()) {
            alignLeft();
        }
        if(joystick.rightTrigger().getAsBoolean()) {
            shooter.forward();
        } else if(joystick.leftTrigger().getAsBoolean()) {
            shooter.reverse();
        } else {
            shooter.stopShooter();
        }
        // if(joystick.rightTrigger().getAsBoolean() && (States.state == "canIntake" || States.state == "intaking")) {
        //     shooter.forward();
        // } else {
        //     shooter.stopShooter();
        // }
        if(buttonBoard.getRawButton(1)) {
            elevator.raiseL3();
        }
        if(buttonBoard.getRawButton(2)) {
            elevator.raiseL2();
        }
        if(buttonBoard.getRawButton(3)) {
            elevator.resetElevator();
        }
        if(joystick.a().getAsBoolean()) {
            elevator.raiseL2();
        } else if(joystick.start().getAsBoolean()) {
            elevator.resetElevator();
        }
        if(joystick.leftBumper().getAsBoolean()) {
            shooter.down();
        } else if(joystick.rightBumper().getAsBoolean()) {
            shooter.up();
        }
    }

    public double getXError() {
        return xController.getPositionError();
    }

    public double getYError() {
        return yController.getPositionError();
    }

    public double getYVelocity() {
        return yController.getSetpoint().velocity;
    }

    public void alignRight() {
        if(limelight.getLock()) {
            appliedSpeed.vyMetersPerSecond = xController.calculate(poseX, -0.1651);
            appliedSpeed.vxMetersPerSecond = yController.calculate(poseY, -0.435);
            appliedSpeed.omegaRadiansPerSecond = rotController.calculate(yaw, 0);
            swerve.adjust(appliedSpeed);
        }
    }

    public void alignLeft() {
        if(limelight.getLock()) {
            appliedSpeed.vyMetersPerSecond = xController.calculate(poseX, 0.1691);
            appliedSpeed.vxMetersPerSecond = yController.calculate(poseY, -0.435);
            appliedSpeed.omegaRadiansPerSecond = rotController.calculate(yaw, 0);
            swerve.adjust(appliedSpeed);
        }
    }

    public void readPeriodicInputs() {
        pose = limelight.getPose();
        poseX = ((-1) * pose[0]);
        poseY = pose[2];
        yaw = ((-1) * pose[4]);
        d = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    }

    @Override
    public void outputTelemetry() {
        //System.out.println(sensor.getVoltage());
    }

    @Override
    public void stop() {}
}
