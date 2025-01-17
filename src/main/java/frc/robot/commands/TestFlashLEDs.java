package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDPreset;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.Timer;;

public class TestFlashLEDs extends Command{
    private LEDs leds;
    public TestFlashLEDs(){
        leds = LEDs.getInstance();
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
        if(flashCount< 1000){
            if (timer.hasElapsed(0.1)) {
                flashCount++;
                timer.restart();
            }

            if (flashCount % 2 == 0) {
                leds.setLEDS(LEDPreset.Solid.kYellow);
            } else {
                leds.setLEDS(LEDPreset.Solid.kBlue);
            }
        }
    }
}