package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    private static LEDs mInstance = null;
    private LEDController ledController;

    private Timer timer = new Timer();
    private int flashCount = 0;
    private boolean flashing = false;
    private ILEDPreset flashColour;
    private int flashDuration;

    // Instance
    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    // Constructor
    private LEDs() {
        ledController = new LEDController(Constants.LEDs.ledPort);// Sets which motor we are using, currently port 9
    }

    // Set the LEDs to be a colour
    public void setLEDS(ILEDPreset colour) {
        ledController.set(colour);
    }

    // Set the LEDs to be Default colour
    public void setLEDSDefault() {
        setLEDS(Constants.LEDs.defaultLED);
    }

    // Flash the LEDs

    // Initialize variables
    public void startFlashing(ILEDPreset colour, int flashSeconds) {
        flashColour = colour;
        flashDuration = flashSeconds;
        flashCount = 0;
        flashing = true;
        timer.reset();
        timer.start();
    }

    // update LED colour every 0.1 seconds
    public void updateFlash() {
        if (!flashing)
            return;

        if (timer.hasElapsed(0.1)) { // Check every 0.1 seconds
            flashCount++;
            timer.reset(); // Restart the timer for the next interval

            if (flashCount / 10 >= flashDuration) { // Stop after flashSeconds
                flashing = false;
                setLEDS(LEDPreset.Solid.kBlack);
                return;
            }

            // Set LEDs
            if (flashCount % 2 == 0) {
                setLEDS(flashColour);
            } else {
                setLEDS(LEDPreset.Solid.kBlack);// OFF
            }
        }
    }
}