
package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
    private TalonFX climbMotor;

    // private DigitalInput limitSwitch;
    private final MedianFilter m_filter;
    // private final DigitalInput kUltrasonicEchoPort;
    // private final DigitalOutput kUltrasonicPingPort;
    // private final Ultrasonic m_ultrasonic;

    public Climb() {
        climbMotor = new TalonFX(7);
        // kUltrasonicEchoPort = new DigitalInput(8);
        // kUltrasonicPingPort = new DigitalOutput(9);
        // m_ultrasonic = new Ultrasonic(9, 8);
        m_filter = new MedianFilter(5);

        // linebreak = new DigitalInput(7);
        // limitSwitch = new DigitalInput(1);
    }

    public void startMotor(double speed) {
        // if (limitSwitch.get()) {
        climbMotor.set(speed);
        // We are going up but limit is not tripped so go at commanded speed
        // } else {
        // System.out.println("LIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMITLIMIT");
        // climbMotor.set(0);
        // // We are going up and limit is tripped so stop
        // }

    }

    public void stopMotor() {
        climbMotor.set(0);
    }

}
