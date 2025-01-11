// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
package frc.robot.modules.gyro;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.PhoenixOdometryThread;

/**
 * 
 */
public class GyroIONavX implements GyroIO {
    private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) PhoenixOdometryThread.ODOMETRY_FREQUENCY);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    /**
     * 
     */
    public GyroIONavX() {
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
