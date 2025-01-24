
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {

    private TalonFX climbMotor;
    private DigitalInput linebreak;
    private DigitalInput limitSwitch;

    public Climb() {
        climbMotor = new TalonFX(7);
        // linebreak = new DigitalInput(7);
        limitSwitch = new DigitalInput(1);

    }

    public void startMotor(double speed) {
        if (limitSwitch.get()) {
            // We are going up and limit is tripped so stop
        } else {
            System.out.println("LIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMIT");
            // We are going up but limit is not tripped so go at commanded speed
        }
        // climbMotor.set(speed);
    }

    public void stopMotor() {
        // climbMotor.set(0);
    }

    // public void linebreak() {
    // if (linebreak.get()) {
    // System.out.println("LINE BROKEN");
    // }
    // }

}
