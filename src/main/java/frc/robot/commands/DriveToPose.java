// package frc.robot.commands;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;

// public class DriveToPose extends Command {
//     private static final double drivekP = 2.0;
//     private static final double drivekD = 0.0;
//     private static final double thetakP = 5.0;
//     private static final double thetakD = 0.0;
//     private static final double driveMaxVelocity = Units.inchesToMeters(150.0);
//     private static final double driveMaxAcceleration = Units.inchesToMeters(95.0);
//     private static final double thetaMaxVelocity = Units.degreesToRadians(360.0);
//     private static final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
//     private static final double driveTolerance = 0.01;
//     private static final double thetaTolerance = Units.degreesToRadians(1.0);
//     private static final double ffMinRadius = 0.2;
//     private static final double ffMaxRadius = 0.8;

//     private final Drive drive;
//     private final Supplier<Pose2d> target;

//     private final ProfiledPIDController driveController = new ProfiledPIDController(drivekP, 0.0, drivekD, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
//     private final ProfiledPIDController thetaController = new ProfiledPIDController(thetakP, 0.0, thetakD, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

//     private Translation2d lastSetpointTranslation = new Translation2d();
//     private double driveErrorAbs = 0.0;
//     private double thetaErrorAbs = 0.0;
//     private boolean running = false;
//     private Supplier<Pose2d> robot = () -> Drive.getInstance().drivetrain.getState().Pose;

//     public boolean isRunning() {
//         return running;
//     }

//     private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
//     private DoubleSupplier omegaFF = () -> 0.0;

//     public DriveToPose(Drive drive, Supplier<Pose2d> target) {
//         this.drive = drive;
//         this.target = target;

//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(drive);
//     }

//     public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
//         this(drive, target);
//         this.robot = robot;
//     }

//     public DriveToPose(
//         Drive drive,
//         Supplier<Pose2d> target,
//         Supplier<Pose2d> robot,
//         Supplier<Translation2d> linearFF,
//         DoubleSupplier omegaFF) {
//         this(drive, target, robot);
//         this.linearFF = linearFF;
//         this.omegaFF = omegaFF;
//     }

//       @Override
//   public void initialize() {
//     Pose2d currentPose = robot.get();
//     ChassisSpeeds fieldVelocity = Drive.getInstance().drivetrain.getState().Speeds;
//     Translation2d linearFieldVelocity =
//         new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
//     driveController.reset(
//         currentPose.getTranslation().getDistance(target.get().getTranslation()),
//         Math.min(
//             0.0,
//             -linearFieldVelocity
//                 .rotateBy(
//                     target
//                         .get()
//                         .getTranslation()
//                         .minus(currentPose.getTranslation())
//                         .getAngle()
//                         .unaryMinus())
//                 .getX()));
//     thetaController.reset(
//         currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
//     lastSetpointTranslation = currentPose.getTranslation();
//   }

//   @Override
//   public void execute() {
//     running = true;
//   }

//   Pose2d currentPose = robot.get();
//   Pose2d targetPose = target.get();

//   // Calculate drive speed
//     double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
//     double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),0.0,1.0);
//     driveErrorAbs = currentDistance;
//     driveController.reset(
//         lastSetpointTranslation.getDistance(targetPose.getTranslation()),
//         driveController.getSetpoint().velocity);
//     double driveVelocityScalar =
//         driveController.getSetpoint().velocity * ffScaler
//             + driveController.calculate(driveErrorAbs, 0.0);
//     if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
//     lastSetpointTranslation =
//         new Pose2d(
//                 targetPose.getTranslation(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
//             .getTranslation();
//     }
// }

