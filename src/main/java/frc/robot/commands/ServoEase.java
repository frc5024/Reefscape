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
        currentAngle = 0;//Can be set to whatever you want, it will be the starting angle
        timer.reset();
        timer.restart();
        servo.setServo(currentAngle);
    }

    @Override
    public void execute() {
        if (currentAngle < finalAngle) {//if the starting angle is less than the final angle do addition
            if (timer.hasElapsed(2 / finalAngle)) {//Sets total time to 2 seconds (ish) can be changed
                currentAngle++;//Updates current angle by 1 additon
                timer.restart();//reset and restart the timer 
                servo.setServo(currentAngle);//Sets to current angle
            }
        } else if (currentAngle > finalAngle) {//if the starting angle is greater than the final angle do subtraction
            if (timer.hasElapsed(2 / finalAngle)) {
                currentAngle--;//Updates current angle by 1 subtraction
                timer.restart();
                servo.setServo(currentAngle);
            }
        }
    }
}
