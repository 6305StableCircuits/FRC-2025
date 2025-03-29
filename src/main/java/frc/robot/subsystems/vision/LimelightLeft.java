package frc.robot.subsystems.vision;

import frc.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightLeft extends Subsystem {

    // Network Table calls to access information directly from the limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
    NetworkTableEntry tx = table.getEntry("tx"); // Horizontal Offset of target from limelight crosshair
    NetworkTableEntry ty = table.getEntry("ty"); // Vertical Offset of target from limelight crosshair
    NetworkTableEntry ta = table.getEntry("ta"); // Percentage of limelight's vision taken up by target's area
    NetworkTableEntry tv = table.getEntry("tv"); // Returns whether or not at least one valid target is in view
    double x,y,area;
    boolean v;
    double[] pose = new double[6];
    double yaw;
    
    // Create a null instance of the Subsystem as well as a method getInstance() which will instantiate an instance upon
    // its first call and return the same instance for subsequent calls, ensuring that we don't end up with duplicate instances
    public static LimelightLeft instance = null;
    public static LimelightLeft getInstance() {
        if(instance == null) {
            instance = new LimelightLeft();
        }
        return instance;
    }
    
    // Accessor method returning true if a valid target is in view and false if one is not
    public boolean getLock() {
        return v;
    }

    // Accessor method returning the target's horizontal offset from the limelight's crosshair as a double
    public double getXOffset() {
        return x;
    }

    // Accessor method returning the target's vertical offset from the limelight's crosshair as a double
    public double getYOffset() {
        return y;
    }

    // Accessor method returning the percentage of the limelight's view taken up by the target's area as a double
    public double getArea() {
        return area;
    }

    public double[] getPose() {
        return pose;
    }

    public double getYaw() {
        return yaw;
    }

    // Void method on a loop pulling updated telemetry to later push to SmartDashboard
    public void readPeriodicInputs() {
        x = this.getLock() ? tx.getDouble(0.0) : 100.0;
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getInteger(0) == 1 ? true : false;
        pose = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        yaw = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    }

    @Override
    public void stop() {}

    // Void method on a loop pushing updated telemetry to SmartDashboard
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("LimelightLeftX", x);
        SmartDashboard.putNumber("LimelightLeftY", y);
        SmartDashboard.putNumber("LimelightLeftArea", area);
        SmartDashboard.putBoolean("LeftLock?", v);
    }
}
