
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;;

public class Climb extends SubsystemBase {

    private TalonFX climbMotor;

    public Climb() {
        climbMotor = new TalonFX(0);
    }

    public void motorController(boolean motorOn) {
        if (motorOn) {
            climbMotor.set(.7);
        } else {
            climbMotor.set(0);
        }
    }

}
