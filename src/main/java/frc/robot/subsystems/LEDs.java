package frc.robot.subsystems;

//Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDController;
import frc.robot.Constants;
import frc.robot.commands.led.FlashLEDs;
import frc.robot.commands.led.SetLEDS;
import frc.robot.commands.led.SetLEDSDefault;

public class LEDs extends SubsystemBase {
    // Variables
    private static LEDs mInstance = null;
    private LEDController ledController;

    // Instance
    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    // Constructor
    private LEDs() {
        ledController = new LEDController(Constants.LEDsConstants.ledPort);// Sets which motor we are using, currently
    }

    // Direct Commands in order to set LEDs to a colour

    // Set the LEDs to be a colour
    public void set(ILEDPreset colour) {
        ledController.set(colour);
    }

    // Set the LEDs to be Default colour
    public void setDefault() {
        set(Constants.LEDsConstants.defaultLED);
    }

    // Command Callers

    // Flash LEDs Command with one colour
    public Command flashCommand(ILEDPreset colour, int flashSeconds) {
        return new FlashLEDs(this, colour, flashSeconds);
    }

    // Flash LEDs Command with two colours
    public Command flashCommand(ILEDPreset colour1, ILEDPreset colour2, int flashSeconds) {
        return new FlashLEDs(this, colour1, colour2, flashSeconds);
    }

    // Set to colour Command
    public Command setCommand(ILEDPreset colour) {
        return new SetLEDS(this, colour);
    }

    // Set LEDs to Default Command
    public Command setDefaultCommand() {
        return new SetLEDSDefault(this);
    }

    public void setLEDSDefault() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setLEDSDefault'");
    }
}