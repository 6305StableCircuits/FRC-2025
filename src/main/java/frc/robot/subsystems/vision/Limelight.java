package frc.robot.subsystems.vision;

import frc.robot.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends Subsystem {

    // Network Table calls to access information directly from the limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // Horizontal Offset of target from limelight crosshair
    NetworkTableEntry ty = table.getEntry("ty"); // Vertical Offset of target from limelight crosshair
    NetworkTableEntry ta = table.getEntry("ta"); // Percentage of limelight's vision taken up by target's area
    NetworkTableEntry tv = table.getEntry("tv"); // Returns whether or not at least one valid target is in view
    double x,y,area;
    boolean v;
    double[] pose;
    
    // Create a null instance of the Subsystem as well as a method getInstance() which will instantiate an instance upon
    // its first call and return the same instance for subsequent calls, ensuring that we don't end up with duplicate instances
    public static Limelight instance = null;
    public static Limelight getInstance() {
        if(instance == null) {
            instance = new Limelight();
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

    // Void method on a loop pulling updated telemetry to later push to SmartDashboard
    public void readPeriodicInputs() {
        x = this.getLock() ? tx.getDouble(0.0) : 100.0;
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getInteger(0) == 1 ? true : false;
        pose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    @Override
    public void stop() {}

    // Void method on a loop pushing updated telemetry to SmartDashboard
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("Lock?", v);
    }
}
