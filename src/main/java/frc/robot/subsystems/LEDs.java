package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Test5024LEDs;
import frc.robot.commands.TestFlashLEDs;

public class LEDs extends SubsystemBase {
    private static LEDs mInstance = null;
    private LEDController ledController;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    private LEDs() {
        ledController = new LEDController(Constants.LEDs.ledPort);// Sets which motor we are using, currently port 9
    }

    // COMMANDS
    public void setLEDS(ILEDPreset colour) {
        ledController.set(colour);
    }

    public Command flashLEDS(ILEDPreset colour, int flashMiliseconds) {
        return new TestFlashLEDs(this, colour, flashMiliseconds);
    }

    public Command E5024LEDS(int flashMiliseconds) {
        return new Test5024LEDs(this, flashMiliseconds);
    }

}