package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;;

public class LEDFlash extends Command {
    private LEDs leds;
    private ILEDPreset colour;
    private int flashSeconds;
    private int flashMiliseconds;

    private Timer totalTime = new Timer();
    private Timer timer = new Timer();
    private int flashCount = 0;

    public LEDFlash(LEDs leds, ILEDPreset colour, int flashSeconds) {
        this.leds = leds;
        this.colour = colour;
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
                leds.set(colour);
            } else {// Else set to black
                leds.set(LEDPreset.Solid.kBlack);
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
