package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Limelight;

public class LEDs extends Subsystem {
    // Instantiate the CANdle with an ID defined in Constants
    CANdle candle = new CANdle(Constants.candleID);
    Limelight limelight = Limelight.getInstance();
    
    // Create a null instance of the Subsystem as well as a method getInstance() which will instantiate an instance upon
    // its first call and return the same instance for subsequent calls, ensuring that we don't end up with duplicate instances
    public static LEDs instance = null;
    public static LEDs getInstance() {
        if(instance == null) {
            instance = new LEDs();
        }
        return instance;
    }
    
    // Configure the CANdle
    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // Sets the LED Strip Type to RGB
        config.brightnessScalar = 0.5; // Sets LEDs to half brightness
        candle.configAllSettings(config);
        candle.setLEDs(255, 0, 0); // Defaults LEDs to red upon init
    }

    // Mutator method for updating the active LED color on the CANdle
    public void setLEDColor(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    // Looping method setting the CANdle orange if the limelight locks on to a target and green if it is centered
    public void update() {
        if(limelight.getXOffset() <= 1.0 && limelight.getXOffset() >= -1.0) {
            candle.setLEDs(0, 255, 0);
        } else if(limelight.getLock()) {
            candle.setLEDs(255, 128, 0);
        } else {
            RainbowAnimation anim = new RainbowAnimation(1.0, 100.0, 8);
            candle.animate(anim);
        }
    }

    @Override
    public void outputTelemetry() {}
 
    @Override
    public void stop() {}
}
