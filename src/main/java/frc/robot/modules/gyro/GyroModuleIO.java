package frc.robot.modules.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 
 */
public interface GyroModuleIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
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
