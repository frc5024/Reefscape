
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {

    private TalonFX climbMotor;
    private DigitalInput linebreak;

    public Climb() {
        climbMotor = new TalonFX(7);
        // linebreak = new DigitalInput(7);
    }

    public void startMotor(double speed) {
        climbMotor.set(speed);
    }

    public void stopMotor() {
        climbMotor.set(0);
    }

    // public void linebreak() {
    // if (linebreak.get()) {
    // System.out.println("LINE BROKEN");
    // }
    // }

}
