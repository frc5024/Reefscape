package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ILEDPreset;
import frc.robot.subsystems.LEDs;;

public class flashLEDS extends Command {
    private LEDs leds;
    private ILEDPreset colour;
    private int flashSeconds;

    private Timer totalTime = new Timer();

    // Contructor takes in LED subsystem, LED Preset, flashSeoncds
    public flashLEDS(LEDs leds, ILEDPreset colour, int flashSeconds) {
        // Subsystem
        this.leds = leds;
        // Preset (Colour)
        this.colour = colour;
        // How long to flash for
        this.flashSeconds = flashSeconds;
    }

    @Override
    public void initialize() {
        // Reset and restart timer
        totalTime.reset();
        totalTime.restart();
        // Tell led subsystem we want to flash
        leds.startFlashing(colour, flashSeconds);
    }

    @Override
    public void execute() {
        // Update the flash
        leds.updateFlash();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // After certain time has elapsed stop command and reset to default
        if (totalTime.hasElapsed(flashSeconds)) {
            leds.setLEDS(Constants.LEDs.defaultLED);
            return true;
        }
        return false;
    }
}
