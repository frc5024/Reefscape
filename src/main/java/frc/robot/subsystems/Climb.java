
package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
    private static Climb mInstance = null;

    private TalonFX climbMotor;

    // private DigitalInput limitSwitch;
    // private final DigitalInput kUltrasonicEchoPort;
    // private final DigitalOutput kUltrasonicPingPort;
    // private final Ultrasonic m_ultrasonic;

    ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    GenericEntry climbSpeed = tab.add("climbSpeed", .35).getEntry();

    private final Ultrasonic m_ultrasonic = new Ultrasonic(5, 4);
    double distanceMillimetres;
    MedianFilter filter = new MedianFilter(5);

    public static Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }

    private Climb() {
        climbMotor = new TalonFX(7);
        // kUltrasonicEchoPort = new DigitalInput(8);
        // kUltrasonicPingPort = new DigitalOutput(9);
        // m_ultrasonic = new Ultrasonic(9, 8);
        tab.addDouble("encoder value", () -> climbMotor.getPosition().getValueAsDouble());

        Ultrasonic.setAutomaticMode(true);
        // linebreak = new DigitalInput(7);
        // limitSwitch = new DigitalInput(1);
    }

    public void startMotor(double speed) {
        // if (limitSwitch.get()) {
        speed = climbSpeed.getDouble(0);
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

    @Override
    public void periodic() {
        distanceMillimetres = m_ultrasonic.getRangeMM();
        double measurement = filter.calculate(distanceMillimetres);
        boolean overThreshold;

        if (measurement >= 100) {
            overThreshold = true;
        } else {
            overThreshold = false;
        }

        SmartDashboard.putBoolean("Over Threshold", overThreshold);
        SmartDashboard.putNumber("Ultrasonic", measurement);
    }
}
