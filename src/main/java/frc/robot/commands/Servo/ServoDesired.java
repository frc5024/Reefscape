package frc.robot.commands.Servo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.ILEDPreset;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ServoTest;

public class ServoDesired extends Command {
    private ServoTest servo;
    private int finalAngle;
    private LEDs leds;
    private ILEDPreset colour;
    private int flashSeconds;

    // Constructor, intakes angle, LED Subsystem, LED Preset, and flashSeconds
    public ServoDesired(int Angle, LEDs leds, ILEDPreset colour, int flashSeconds) {
        servo = ServoTest.getInstance();
        finalAngle = Angle;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        servo.setServo(finalAngle);
        leds.flashCommand(colour, flashSeconds);
    }
}
