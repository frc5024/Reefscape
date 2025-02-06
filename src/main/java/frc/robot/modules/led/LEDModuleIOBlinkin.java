package frc.robot.modules.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.lib.blinkin.BlinkinPreset;
import frc.lib.blinkin.IBlinkinPreset;

/**
 * 
 */
public class LEDModuleIOBlinkin implements LEDModuleIO {
    private final int CHANNEL_ID = 9;
    private final Spark spark;

    /**
     * 
     */
    public LEDModuleIOBlinkin() {
        this.spark = new Spark(CHANNEL_ID);
    }

    @Override
    public void set(IBlinkinPreset preset) {
        set(preset.getValue());
    }

    /**
     * 
     */
    public void set(double value) {
        this.spark.set(value);
    }

    @Override
    public void solidAqua() {
        this.set(BlinkinPreset.Solid.kAqua);
    }

    @Override
    public void solidBlack() {
        this.set(BlinkinPreset.Solid.kBlack);
    }

    @Override
    public void solidBlue() {
        this.set(BlinkinPreset.Solid.kBlue);
    }

    @Override
    public void solidGreen() {
        this.set(BlinkinPreset.Solid.kGreen);
    }

    @Override
    public void solidRed() {
        this.set(BlinkinPreset.Solid.kRed);
    }

    @Override
    public void solidWhite() {
        this.set(BlinkinPreset.Solid.kWhite);
    }

    @Override
    public void solidYellow() {
        this.set(BlinkinPreset.Solid.kYellow);
    }

    @Override
    public void strobeAqua() {
        this.set(BlinkinPreset.Strobe.kAqua);
    }

    @Override
    public void strobeWhite() {
        this.set(BlinkinPreset.Strobe.kWhite);
    }
}
