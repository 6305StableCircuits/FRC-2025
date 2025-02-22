package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

// Class for storing various Constants accessed across the codebase

public class Constants {
    public static final int candleID = 44;
    public static final double robotWidth = 0;
    public static RobotConfig robotConfig;
    static {
        try {
            Constants.robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static final int hunterID = 10;
    public static final int garrettID = 9;
}
