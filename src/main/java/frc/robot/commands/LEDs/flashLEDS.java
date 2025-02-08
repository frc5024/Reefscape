package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;;

public class FlashLEDS extends Command {
    private LEDs leds;
    private ILEDPreset colour1;
    private ILEDPreset colour2;
    private int flashSeconds;
    private int flashMiliseconds;

    private Timer totalTime = new Timer();
    private Timer timer = new Timer();
    private int flashCount = 0;

    public FlashLEDS(LEDs leds, ILEDPreset colour, int flashSeconds) {
        this(leds, colour, LEDPreset.Solid.kBlack, flashSeconds);
    }

    public FlashLEDS(LEDs leds, ILEDPreset colour1, ILEDPreset colour2, int flashSeconds) {
        this.leds = leds;
        this.colour1 = colour1;
        this.colour2 = colour2;
        this.flashSeconds = flashSeconds;
        flashMiliseconds = flashSeconds * 100;
    }

    @Override
    public void initialize() {
        flashCount = 0;// Resets count just to be safe
        timer.reset();// Resets timer
        timer.restart();// Starts timer
    }

    @Override
    public void execute() {
        if (flashCount < flashMiliseconds) {
            if (timer.hasElapsed(0.1)) {// Number in brackets is in seconds
                flashCount++;
                timer.restart();
            }

            if (flashCount % 2 == 0) {// If count is even set to the colour
                leds.set(colour1);
            } else {// Else set to colour 2
                leds.set(colour2);
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // After certain time has elapsed stop command and reset to default
        if (totalTime.hasElapsed(flashSeconds)) {
            leds.set(Constants.LEDs.defaultLED);
            return true;
        }
        return false;
    }
}