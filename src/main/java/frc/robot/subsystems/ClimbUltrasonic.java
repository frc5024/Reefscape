package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbUltrasonic extends SubsystemBase {
    private static ClimbUltrasonic mInstance = null;

    private final Ultrasonic m_ultrasonic = new Ultrasonic(9, 8);
    double distanceMillimetres;

    public static ClimbUltrasonic getInstance() {
        if (mInstance == null) {
            mInstance = new ClimbUltrasonic();
        }
        return mInstance;
    }

    public void ultrasonic() {
        distanceMillimetres = m_ultrasonic.getRangeMM();
        // double filteredMeasurement = m_filter.calculate(measurement);
        System.out.println(distanceMillimetres);
    }

    @Override
    public void periodic() {

    }
}
