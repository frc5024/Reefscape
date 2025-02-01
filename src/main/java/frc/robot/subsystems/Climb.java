
package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {
    private static Climb mInstance = null;

    private TalonFX climbMotor;
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
        tab.addDouble("encoder value", () ->climbMotor.getPosition().getValueAsDouble());
        
    }

    public void startMotor(double speed) {
       
        speed = climbSpeed.getDouble(0);
        climbMotor.set(speed);
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
