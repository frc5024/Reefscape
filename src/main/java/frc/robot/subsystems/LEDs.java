package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDController;
import frc.robot.Constants;
import frc.robot.commands.LEDs.FlashLEDS;
import frc.robot.commands.LEDs.SetLEDS;
import frc.robot.commands.LEDs.SetLEDSDefault;

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
    public void set(ILEDPreset colour) {
        ledController.set(colour);
    }

    // Set the LEDs to be Default colour
    public void setDefault() {
        set(Constants.LEDs.defaultLED);
    }

    public Command flashCommand(ILEDPreset colour, int flashSeconds) {
        return new FlashLEDS(this, colour, flashSeconds);
    }

    public Command flashCommand(ILEDPreset colour1, ILEDPreset colour2, int flashSeconds) {
        return new FlashLEDS(this, colour1, colour2, flashSeconds);
    }

    public Command setCommand(ILEDPreset colour) {
        return new SetLEDS(this, colour);
    }

    public Command setDefaultCommand() {
        return new SetLEDSDefault(this);
    }

    public void setLEDSDefault() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setLEDSDefault'");
    }
}