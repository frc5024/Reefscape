package frc.lib.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDController {

    private Spark mSpark;

    public LEDController(final int channel) {
        mSpark = new Spark(channel);
    }

    public void set(ILEDPreset preset) {
        set(preset.value());
    }

    public void set(double value) {
        mSpark.set(value);
    }
}