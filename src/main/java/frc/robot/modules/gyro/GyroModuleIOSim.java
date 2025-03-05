package frc.robot.modules.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.SparkUtil;

/**
 * 
 */
public class GyroModuleIOSim implements GyroModuleIO {
    private final GyroSimulation gyroSimulation;

    /**
     * 
     */
    public GyroModuleIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
                this.gyroSimulation != null,
                this.gyroSimulation != null ? this.gyroSimulation.getGyroReading() : new Rotation2d(),
                this.gyroSimulation != null ? Units.degreesToRadians(
                        gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond)) : 0.0);

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = this.gyroSimulation != null ? gyroSimulation.getCachedGyroReadings()
                : new Rotation2d[0];
    }

}
