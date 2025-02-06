package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ServoTest extends SubsystemBase {
    private static ServoTest mInstance = null;
    private Servo exampleServo;

    public static ServoTest getInstance() {
        if (mInstance == null) {
            mInstance = new ServoTest();
        }
        return mInstance;
    }

    private ServoTest() {
        exampleServo = new Servo(Constants.Servo.servoPort);
    }

    public void setServo(double angle) {// Sets to angle imputed, used in commands
        exampleServo.setAngle(angle);
    }
}
