package frc.robot;

public class States {
    public static String state = "canIntake";
    public static boolean manualMode = true;

    public static void setState(String input) {
        state = input;
    }

    public static void setMode(boolean input) {
        manualMode = input;
    }
}
