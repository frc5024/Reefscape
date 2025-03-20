package frc.robot.modules.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 
 */
public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public SwerveModuleIOData data = new SwerveModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, false, false,
                Rotation2d.kZero, 0.0, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0);

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public record SwerveModuleIOData(
            boolean driveConnected,
            double drivePositionRad,
            double driveVelocityRadPerSec,
            double driveAppliedVolts,
            double driveSupplyCurrentAmps,
            double driveTorqueCurrentAmps,

            boolean turnConnected,
            boolean turnEncoderConnected,
            Rotation2d turnAbsolutePosition,
            double turnAbsolutePositionDeg,
            Rotation2d turnPosition,
            double turnPositionDeg,
            double turnVelocityRadPerSec,
            double turnAppliedVolts,
            double turnSupplyCurrentAmps,
            double turnTorqueCurrentAmps) {
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    /**
     * 
     */
    public default void resetDrivePID() {
    }

    /**
     * 
     */
    public default void setBrakeMode(boolean enabled) {
    }

    /**
     * 
     */
    public default void setDrivePID(double kP, double kI, double kD) {
    }

    /**
     * Run the drive motor at the specified velocity.
     */
    public default void setDriveSetpoint(double velocityRadPerSec) {
    }

    /**
     * Run the drive motor at the specified open loop value.
     */
    public default void setDriveVoltage(double output) {
    }

    /**
     * Run the turn motor to the specified rotation.
     */
    public default void setTurnSetpoint(Rotation2d rotation) {
    }

    /**
     * Run the turn motor at the specified open loop value.
     */
    public default void setTurnVoltage(double output) {
    }

    /**
     * 
     */
    public default void setTurnPID(double kP, double kI, double kD) {
    }
}
