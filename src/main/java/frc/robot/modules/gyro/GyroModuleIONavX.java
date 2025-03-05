package frc.robot.modules.gyro;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.filter.Debouncer;
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

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    /**
     * 
     */
    public GyroModuleIONavX() {
        this.yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this::getScaledYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
                connectedDebouncer.calculate(this.gyro.isConnected()),
                Rotation2d.fromRotations(getScaledYaw()),
                Units.rotationsToRadians(this.gyro.getVelocityZ()));

        inputs.odometryYawTimestamps = this.yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = this.yawPositionQueue.stream()
                .map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        this.yawTimestampQueue.clear();
        this.yawPositionQueue.clear();
    }

    /**
     * Fix for NavX rotational drift
     */
    public double getScaledYaw() {
        double angle = (this.gyro.getAngle() * GyroContants.SCALE_VALUE) % 360.0;

        if (angle > 180) {
            angle = angle - 360;
        }

        return angle;
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-getScaledYaw());
    }

    @Override
    public void zeroHeading() {
        this.gyro.reset();
    }
}
