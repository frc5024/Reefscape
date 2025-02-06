package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTest;

public class ServoEase extends Command {
    private ServoTest servo;
    private int finalAngle;
    private int currentAngle;
    private Timer timer = new Timer();

    public ServoEase(int Angle) {
        servo = ServoTest.getInstance();
        finalAngle = Angle;
    }

    @Override
    public void initialize() {
        currentAngle = 0;
        timer.reset();
        timer.restart();
        servo.setServo(currentAngle);
    }

    @Override
    public void execute() {// Checks where the starting point is (Larger or Smaller), changes the servo
                           // angle
        if (currentAngle < finalAngle) {
            if (timer.hasElapsed(2 / finalAngle)) {// Total time will be around 2 seconds, as the "time" in the bracket
                                                   // is in seconds
                currentAngle++;
                timer.restart();
                servo.setServo(currentAngle);
            }
        } else if (currentAngle > finalAngle) {
            if (timer.hasElapsed(2 / finalAngle)) {
                currentAngle--;
                timer.restart();
                servo.setServo(currentAngle);
            }
        }
    }
}
