package frc.robot.subsystems;

import frc.lib.blinkin.IBlinkinPreset;
import frc.robot.Robot;
import frc.robot.modules.led.LEDModuleIO;
import frc.robot.modules.led.LEDModuleIOBlinkin;
import frc.robot.modules.led.LEDModuleIOSim;

public class LEDSubsystem {
    private static LEDSubsystem mInstance = null;
    private final LEDModuleIO ledModuleIO;

    /**
     * 
     */
    private LEDSubsystem(LEDModuleIO ledModuleIO) {
        this.ledModuleIO = ledModuleIO;
    }

    /**
     * 
     */
    public static LEDSubsystem getInstance() {
        if (mInstance == null) {
            if (Robot.isReal()) {
                mInstance = new LEDSubsystem(new LEDModuleIOBlinkin());
            } else {
                mInstance = new LEDSubsystem(new LEDModuleIOSim());
            }
        }

        return mInstance;
    }

    /**
     * 
     */
    public void set(IBlinkinPreset preset) {
        this.ledModuleIO.set(preset);
    }

    /**
     * 
     */
    public void solidAqua() {
        this.ledModuleIO.solidAqua();
    }

    public void solidBlack() {
        this.ledModuleIO.solidBlack();
    }

    public void solidBlue() {
        this.ledModuleIO.solidBlue();
    }

    public void solidGreen() {
        this.ledModuleIO.solidGreen();
    }

    public void solidRed() {
        this.ledModuleIO.solidRed();
    }

    public void solidWhite() {
        this.ledModuleIO.solidWhite();
    }

    public void solidYellow() {
        this.ledModuleIO.solidYellow();
    }

    public void strobeAqua() {
        this.ledModuleIO.strobeAqua();
    }

    public void strobeWhite() {
        this.ledModuleIO.strobeWhite();
    }
}
