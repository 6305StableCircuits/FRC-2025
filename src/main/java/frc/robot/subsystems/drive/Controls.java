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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.subsystems.vision.LimelightLeft;
import frc.robot.subsystems.vision.LimelightRight;

public class Controls extends Subsystem {

    CommandXboxController joystick = new CommandXboxController(0);
    GenericHID buttonBoard = new GenericHID(1);
    Drive swerve = Drive.getInstance();
    LimelightRight limelightRight = LimelightRight.getInstance();
    LimelightLeft limelightLeft = LimelightLeft.getInstance();
    Elevator elevator = Elevator.getInstance();
    Shooter shooter = Shooter.getInstance();
    //L2Left l2Left = new L2Left(this);

    //ProfiledPIDController controller = new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(5, 10));
    //HolonomicDriveController controller = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
    double[] poseRight = new double[6];
    double[] poseLeft = new double[6];
    double d;
    ChassisSpeeds vel;
    double poseXRight,poseYRight,yawRight,poseXLeft,poseYLeft,yawLeft;
    Pigeon2 pigeon = new Pigeon2(30);
    Timer timer = new Timer();
    Rotation2d rot,rot2;
    Pose2d desPos,curPos;
    Translation2d mid;
    TrajectoryConfig config;
    Trajectory trajectory;
    boolean trajectoryGenerated = false;
    PathPlannerPath path;
    int cR = 0;
    int cL = 0;
    int negRight = 0;
    int negLeft = 0;
    //1.25 0.8 0
    //0.95 0.05 0

    //1.25 0.8 0
    //0.95 0.05 0
    //0.02 0 0

    //lower kP increase kI probably
    ProfiledPIDController xController = new ProfiledPIDController(0.4, 0.2,0, new TrapezoidProfile.Constraints(1, 0.25));
    ProfiledPIDController yController = new ProfiledPIDController(0.3, 0.1, 0, new TrapezoidProfile.Constraints(1, 0.25));
    ProfiledPIDController rotController = new ProfiledPIDController(0.02, 0, 0, new TrapezoidProfile.Constraints(3, 1));

    SimpleMotorFeedforward xFeedforward = new SimpleMotorFeedforward(0, 1.25, 1.5);
    SimpleMotorFeedforward yFeedforward = new SimpleMotorFeedforward(0, 1.25, 1.5);

    ChassisSpeeds appliedSpeed = new ChassisSpeeds();

    double prevVel = 0;
    double velX = 0;
    double velY = 0;
    double velOmega = 0;

    public boolean teleop = false;

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
        xController.setTolerance(.04);
        yController.setTolerance(.05);
        rotController.setTolerance(4);
        rotController.reset(0);
    }

    public void setTELEOP(boolean tel) {
        teleop = tel;
    }

    public void update() {
        swerve.swerve(joystick);
        if(beamBreak.get() == false) {
            if(!limelightLeft.getLock() && !limelightRight.getLock()) {
                States.setState("coralHeld");
            } else if((limelightRight.getLock() && ((Math.abs(poseXRight - 0.470)) < 0.03) && ((Math.abs(poseYRight + 0.22))) < 0.05) || (limelightLeft.getLock() && ((Math.abs(poseXLeft + 0.44)) < 0.03) && ((Math.abs(poseYLeft + 0.22)) < 0.05))) {
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
            leds.setLEDColor(255, 255, 255);
        } else if(States.state == "Fire!") {
            leds.setLEDColor(0, 255, 0);
        } else {
            leds.setLEDColor(255, 0, 0);
            xController.reset(0);
            yController.reset(0);
            rotController.reset(0);
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
        if(joystick.a().getAsBoolean()) {
            elevator.blip();
        }
        if(teleop) {
            if(buttonBoard.getRawButton(6) && States.state == "canIntake") {
                shooter.intake();
            } else if(buttonBoard.getRawButton(9)) {
                shooter.forward();
            } else {
                shooter.stopShooter();
            }
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
        if(cR == 0) {
            if((0.450 - poseXRight) < 0) {
                cR = -1;
            } else if((0.450 - poseXRight) > 0) {
                cR = 1;
            }
        }

        if(xController.atGoal()) {
            cR = 0;
        }

        System.out.println(xController.getSetpoint().velocity);
        velY = (-1) * (xController.calculate(poseXRight, 0.450) + (xFeedforward.calculate(cR * xController.getSetpoint().velocity))); // -0.1651 | -0.1905 // -0.41
        velX = (yController.calculate(poseYRight, -0.25) - yFeedforward.calculate(yController.getSetpoint().velocity)); // -0.2
        velOmega = (-1) * rotController.calculate(yawRight, -4);
        swerve.adjust(velX, velY, velOmega);
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
        if(cL == 0) {
            if((-0.42 - poseXLeft) < 0) {
                cL = 1;
            } else if((-0.42 - poseXLeft) > 0) {
                cL = -1;
            }
        }

        if(xController.atGoal()) {
            cL = 0;
        }
        
        velY = ((-1) * ((xController.calculate(poseXLeft, -0.42)) + xFeedforward.calculate(cL * xController.getSetpoint().velocity))); // -0.1651 | -0.1905 // -0.41
        velX = (yController.calculate(poseYLeft, -0.25) - yFeedforward.calculate(yController.getSetpoint().velocity)); // -0.2
        velOmega = (-1) * rotController.calculate(yawLeft, 0);
        swerve.adjust(velX, velY, velOmega);
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
        poseRight = limelightRight.getPose();
        poseXRight = (poseRight[0]);
        poseYRight = poseRight[2];
        yawRight = limelightRight.getYaw();
        //System.out.println(yawLeft);

        poseLeft = limelightLeft.getPose();
        poseXLeft = (poseLeft[0]);
        poseYLeft = poseLeft[2];
        yawLeft = limelightLeft.getYaw();
        //System.out.println(poseXLeft);
        // d = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
