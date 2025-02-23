package frc.robot.modules.gyro;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.GyroContants;
import frc.robot.utils.PhoenixOdometryThread;

/**
 * 
 */
public class GyroModuleIONavX implements GyroModuleIO {
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, (byte) PhoenixOdometryThread.ODOMETRY_FREQUENCY);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    /**
     * 
     */
    public GyroModuleIONavX() {
        this.yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this::getScaledYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = this.gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-this.getScaledYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-this.getScaledYaw());

        inputs.odometryYawTimestamps = this.yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = this.yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        this.yawTimestampQueue.clear();
        this.yawPositionQueue.clear();
    }

    /**
     * Fix for NavX rotational drift
     */
    private double getScaledYaw() {
        double angle = (this.gyro.getAngle() * GyroContants.SCALE_VALUE) % 360.0;

        if (angle > 180) {
            angle = angle - 360;
        }

        return angle;
    }

    @Override
    public void zeroHeading() {
        this.gyro.reset();
    }
}
