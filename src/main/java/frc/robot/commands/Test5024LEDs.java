package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDPreset;
import frc.robot.subsystems.LEDs;;

//A for fun thing I did
public class Test5024LEDs extends Command {
    private LEDs leds;
    private int flashMiliseconds;

    public Test5024LEDs(LEDs leds, int flashMiliseconds) {
        this.leds = leds;
        this.flashMiliseconds = flashMiliseconds;
    }

    private Timer timer = new Timer();
    private int flashCount = 0;

    @Override
    public void initialize() {
        flashCount = 0;
        timer.reset();
        timer.restart();
    }

    @Override
    public void execute() {
        if (flashCount < flashMiliseconds) {
            if (timer.hasElapsed(0.1)) {
                flashCount++;
                timer.restart();
            }

            if (flashCount % 2 == 0) {// If counter is even
                leds.setLEDS(LEDPreset.Solid.kYellow);// set colour to yellow
            } else {// else
                leds.setLEDS(LEDPreset.Solid.kBlue);// set colour to blue
            }
        }
    }
}