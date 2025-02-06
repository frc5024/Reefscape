package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTest;

public class ServoDesired extends Command {
    private ServoTest servo;
    private int finalAngle;

    public ServoDesired(int Angle) {// Gets angle imputed
        servo = ServoTest.getInstance();
        finalAngle = Angle;
    }

    @Override
    public void execute() {// Commands Servo to go to angle imputed in RobotContainer
        servo.setServo(finalAngle);// Does it very quickly use Ease for a slower one (or quicker one it can be set
                                   // to any amount of time)
    }
}
