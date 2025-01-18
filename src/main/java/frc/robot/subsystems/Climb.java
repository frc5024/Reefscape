
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
        climbMotor = new TalonFX(41);

        // if (linebreak.get()) {
        // System.out.println("LINE BROKEN");
        // }
    }

    public void startMotor() {
        climbMotor.set(.1);
    }

    public void stopMotor() {
        climbMotor.set(0);
    }

}
