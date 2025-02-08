package frc.robot.commands.Servo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ServoTest;

public class ServoDesired extends Command {
    private ServoTest servo;
    private int finalAngle;
    private LEDs leds;
    private ILEDPreset colour1;
    private ILEDPreset colour2;
    private int flashSeconds;
    private Timer totalTime = new Timer();

    // Constructor, intakes angle, LED Subsystem, LED Preset colour 1, and
    // flashSeconds
    public ServoDesired(int Angle, LEDs leds, ILEDPreset colour1, int flashSeconds) {
        this(Angle, leds, colour1, LEDPreset.Solid.kBlack, flashSeconds);
    }

    // Constructor, intakes angle, LED Subsystem, LED Preset colour #1, LED Preset
    // colour #2, and flashSeconds
    public ServoDesired(int Angle, LEDs leds, ILEDPreset colour1, ILEDPreset colour2, int flashSeconds) {
        servo = ServoTest.getInstance();
        finalAngle = Angle;
        this.leds = leds;
        this.colour1 = colour1;
        this.colour2 = colour2;
        this.flashSeconds = flashSeconds;
    }

    @Override
    public void initialize() {
        leds.flashCommand(colour1, colour2, flashSeconds).schedule();
        totalTime.reset();
        totalTime.restart();
    }

    @Override
    public void execute() {
        servo.setServo(finalAngle);
        System.out.println("Servo");
        leds.flashCommand(colour1, colour2, flashSeconds);
        System.out.println("LED");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // After certain time has elapsed stop command and reset to default
        if (totalTime.hasElapsed(flashSeconds)) {
            leds.set(Constants.LEDs.defaultLED);
            return true;
        }
        return false;
    }
}
