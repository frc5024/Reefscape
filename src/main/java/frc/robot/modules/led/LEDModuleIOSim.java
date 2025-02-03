package frc.robot.modules.led;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.blinkin.IBlinkinPreset;

public class LEDModuleIOSim implements LEDModuleIO {
    /**
     * 
     */
    public enum Effect {
        SOLID, STROBE
    }

    // Constants
    private final int CHANNEL_ID = 3;
    private final int STRIP_LENGTH = 250;

    // Hardware
    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer addressableLEDBuffer;

    // Variables
    private final Timer timer;
    private TimerTask timerTask;
    private int ledIndex;

    /**
     * 
     */
    public LEDModuleIOSim() {
        this.addressableLED = new AddressableLED(CHANNEL_ID);
        this.addressableLEDBuffer = new AddressableLEDBuffer(STRIP_LENGTH);

        this.timer = new Timer();
        this.ledIndex = 0;

        this.start();
    }

    @Override
    public void set(IBlinkinPreset preset) {
    }

    /**
     * 
     */
    public void set(Color color) {
        for (int i = 0; i < this.addressableLEDBuffer.getLength(); i++) {
            this.addressableLEDBuffer.setLED(i, color);
        }

        this.addressableLED.setData(this.addressableLEDBuffer);
    }

    /**
     * 
     */
    public void set(Effect effect, Object colorObject) {
        try {
            Color color = (Color) colorObject;
            set(effect, color);
        } catch (Exception e) {
        }
    }

    /**
     * 
     */
    public void set(Effect effect, Color color) {
        this.ledIndex = 0;
        if (this.timerTask != null)
            this.timerTask.cancel();

        switch (effect) {
            case STROBE:
                this.timerTask = new TimerTask() {
                    @Override
                    public void run() {
                        strobe(color);
                    }
                };

                this.timer.scheduleAtFixedRate(this.timerTask, 0, 100);
                break;

            default:
                for (int i = 0; i < this.addressableLEDBuffer.getLength(); i++) {
                    this.addressableLEDBuffer.setLED(i, color);
                }

                this.addressableLED.setData(this.addressableLEDBuffer);
                break;
        }
    }

    /**
     * 
     */
    private void set(int r, int g, int b) {
        for (int i = 0; i < this.addressableLEDBuffer.getLength(); i++) {
            this.addressableLEDBuffer.setRGB(i, r, g, b);
        }

        this.addressableLED.setData(this.addressableLEDBuffer);
    }

    @Override
    public void solidBlack() {
        this.set(Effect.SOLID, Color.kBlack);
    }

    @Override
    public void solidBlue() {
        this.set(Effect.SOLID, Color.kBlue);
    }

    @Override
    public void solidGreen() {
        this.set(Effect.SOLID, Color.kGreen);
    }

    @Override
    public void solidRed() {
        this.set(Effect.SOLID, Color.kRed);
    }

    @Override
    public void solidYellow() {
        this.set(Effect.SOLID, Color.kYellow);
    }

    /**
    * 
    */
    public void start() {
        this.addressableLED.setLength(this.addressableLEDBuffer.getLength());
        this.addressableLED.setData(this.addressableLEDBuffer);

        this.addressableLED.start();
    }

    /**
     * 
     */
    public void stop() {
        this.addressableLED.stop();
        this.timer.cancel();
    }

    /**
     * 
     */
    private void strobe(Color color) {
        this.ledIndex++;

        for (int i = 0; i < this.addressableLEDBuffer.getLength(); i++) {
            this.addressableLEDBuffer.setLED(i, this.ledIndex % 2 == 0 ? Color.kBlack : color);
        }

        this.addressableLED.setData(this.addressableLEDBuffer);
    }
}
