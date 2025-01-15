package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Constants;

public class Candle extends Subsystem {
    // Instantiate the CANdle with an ID defined in Constants
    CANdle candle = new CANdle(Constants.candleID);
    
    // Create a null instance of the Subsystem as well as a method getInstance() which will instantiate an instance upon
    // its first call and return the same instance for subsequent calls, ensuring that we don't end up with duplicate instances
    public static Candle instance = null;
    public static Candle getInstance() {
        if(instance == null) {
            instance = new Candle();
        }
        return instance;
    }
    
    // Configure the CANdle
    public Candle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // Sets the LED Strip Type to RGB
        config.brightnessScalar = 0.5; // Sets LEDs to half brightness
        candle.configAllSettings(config);
        candle.setLEDs(255, 255, 255); // Defaults LEDs to white upon init
    }

    // Mutator method for updating the active LED color on the CANdle
    public void setLEDColor(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
