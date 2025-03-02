package frc.robot.modules.led;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.blinkin.IBlinkinPreset;

/**
 * 
 */
public interface LEDModuleIO {
    @AutoLog
    public static class LEDIOInputs {
        public boolean connected = false;
    }

    /**
     * 
     */
    public default void updateInputs(LEDIOInputs inputs) {
    }

    /**
     * 
     */
    public default void set(IBlinkinPreset preset) {
    }

    /**
     * 
     */
    public default void solidAqua() {
    }

    public default void solidBlack() {
    }

    public default void solidBlue() {
    }

    public default void solidGreen() {
    }

    public default void solidRed() {
    }

    public default void solidWhite() {
    }

    public default void solidYellow() {
    }

    public default void strobeAqua() {
    }

    public default void strobeWhite() {
    }
}
