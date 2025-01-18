package frc.robot.modules.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.SparkUtil;

/**
 * 
 */
public class GyroModuleIOSim implements GyroModuleIO {
    private final GyroSimulation gyroSimulation;

    /**
     * 
     * @param gyroSimulation
     */
    public GyroModuleIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }

}
