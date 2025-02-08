package frc.robot.commands.Servo;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTest;

public class ServoDesired extends Command {
    private ServoTest servo;
    private int finalAngle;

    public ServoDesired(int Angle) {
        servo = ServoTest.getInstance();
        finalAngle = Angle;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        servo.setServo(finalAngle);
    }
}
