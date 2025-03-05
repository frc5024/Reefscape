package frc.robot.modules.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 
 */
public interface GyroModuleIO {
    @AutoLog
    public class GyroIOInputs {
        public GyroIOData data = new GyroIOData(false, Rotation2d.kZero, 0);
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public record GyroIOData(
            boolean connected,
            Rotation2d yawPosition,
            double yawVelocityRadPerSec) {
    }

    /**
     * 
     */
    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default Rotation2d getYaw() {
        return new Rotation2d();
    }

    /**
     * 
     */
    public default void zeroHeading() {
    }
}
