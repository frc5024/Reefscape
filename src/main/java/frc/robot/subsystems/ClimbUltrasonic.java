package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbUltrasonic extends SubsystemBase {
    private static ClimbUltrasonic mInstance = null;

    private final Ultrasonic m_ultrasonic = new Ultrasonic(5, 4);
    double distanceMillimetres;
    MedianFilter filter = new MedianFilter(5);

    public ClimbUltrasonic() {
        Ultrasonic.setAutomaticMode(true);
        // m_ultrasonic.setEnabled(true);
    }

    public static ClimbUltrasonic getInstance() {
        if (mInstance == null) {
            mInstance = new ClimbUltrasonic();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        distanceMillimetres = m_ultrasonic.getRangeMM();
        double measurement = filter.calculate(distanceMillimetres);
        boolean overThreshold;
        // double filteredMeasurement = m_filter.calculate(measurement);
        if (measurement >= 100) {
            overThreshold = true;
        }
        else {
            overThreshold = false;
        }

        SmartDashboard.putBoolean("Over Threshold", overThreshold);
        SmartDashboard.putNumber("Ultrasonic", measurement);
        
    }
}
