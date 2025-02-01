package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ILEDPreset;
import frc.robot.subsystems.LEDPreset;
import frc.robot.subsystems.LEDs;;

public class TestFlashLEDs extends Command {
    private LEDs leds;
    private ILEDPreset colour;
    private int flashMiliseconds;

    public TestFlashLEDs(LEDs leds, ILEDPreset colour, int flashMiliseconds) {
        this.leds = leds;
        this.colour = colour;
        this.flashMiliseconds = flashMiliseconds;
    }

    private Timer timer = new Timer();
    private int flashCount = 0;

    @Override
    public void initialize() {
        flashCount = 0;//Resets count just to be safe
        timer.reset();//Resets timer
        timer.restart();//Starts timer
    }

    @Override
    public void execute() {
        if (flashCount < flashMiliseconds) {
            if (timer.hasElapsed(0.1)) {//Number in brackets is in seconds
                flashCount++;//Updates the LED every 0.1 seconds
                timer.restart();
            }

            if (flashCount % 2 == 0) {//If count is even set to the colour
                leds.setLEDS(colour);
            } else {//Else set to black
                leds.setLEDS(LEDPreset.Solid.kBlack);
            }
        }
    }
}