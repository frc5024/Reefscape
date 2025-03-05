package frc.robot.modules.gyro;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.PhoenixOdometryThread;

/**
 * 
 */
public class GyroModuleIOPigeon2 implements GyroModuleIO {
    private final Pigeon2 pigeon = new Pigeon2(0, "rio");
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    /**
     * 
     */
    public GyroModuleIOPigeon2() {
        this.pigeon.getConfigurator().apply(new Pigeon2Configuration());
        this.pigeon.getConfigurator().setYaw(0.0);
        this.pigeon.optimizeBusUtilization();

        this.yaw.setUpdateFrequency(PhoenixOdometryThread.ODOMETRY_FREQUENCY);
        this.yawVelocity.setUpdateFrequency(50.0);

        this.yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this.pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
                BaseStatusSignal.isAllGood(yaw, yawVelocity),
                Rotation2d.fromDegrees(yaw.getValueAsDouble()),
                Units.degreesToRadians(yawVelocity.getValueAsDouble()));

        inputs.odometryYawTimestamps = this.yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = this.yawPositionQueue.stream()
                .map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        this.yawTimestampQueue.clear();
        this.yawPositionQueue.clear();
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(this.pigeon.getYaw().getValueAsDouble());
    }
}
