package frc.robot.modules.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.Swerve;

/**
 * 
 */
public class SwerveModule {
    private final SwerveModuleIO swerveModuleIO;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /**
     * 
     */
    public SwerveModule(SwerveModuleIO swerveModuleIO, int index) {
        this.swerveModuleIO = swerveModuleIO;
        this.index = index;

        driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + Integer.toString(index) + ".",
                AlertType.kError);
    }

    /**
     * 
     */
    public void periodic() {
        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        this.odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * (Swerve.chosenModule.wheelDiameter / 2);
            Rotation2d angle = inputs.odometryTurnPositions[i];
            this.odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        this.driveDisconnectedAlert.set(!inputs.driveConnected);
        this.turnDisconnectedAlert.set(!inputs.turnConnected);
        this.turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        double velocityRadPerSec = state.speedMetersPerSecond / (Swerve.chosenModule.wheelDiameter / 2);
        Logger.recordOutput("John/sMPS", state.speedMetersPerSecond);
        Logger.recordOutput("John/wD", Swerve.chosenModule.wheelDiameter / 2);
        Logger.recordOutput("John/vRadPS", velocityRadPerSec);

        swerveModuleIO.runDriveVelocity(velocityRadPerSec);
        swerveModuleIO.runTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        swerveModuleIO.runDriveOpenLoop(output);
        swerveModuleIO.runTurnPosition(new Rotation2d());
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        swerveModuleIO.runDriveOpenLoop(0.0);
        swerveModuleIO.runTurnOpenLoop(0.0);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getAngle() {
        return this.inputs.turnPosition;
    }

    /**
     * 
     */
    public int getIndex() {
        return this.index;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return this.inputs.drivePositionRad * (Swerve.chosenModule.wheelDiameter / 2);
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * (Swerve.chosenModule.wheelDiameter / 2);
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Returns the module positions received this cycle.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     */
    public double[] getOdometryTimestamps() {
        return this.inputs.odometryTimestamps;
    }

    /**
     * Returns the module position in radians.
     */
    public double getWheelRadiusCharacterizationPosition() {
        return this.inputs.drivePositionRad;
    }

    /**
     * Returns the module velocity in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(this.inputs.driveVelocityRadPerSec);
    }

    /**
     * 
     */
    public void resetDrivePID() {
        this.swerveModuleIO.resetDrivePID();
    }

    /**
     * 
     */
    public void setDrivePID(double kP, double kI, double kD) {
        this.swerveModuleIO.setDrivePID(kP, kI, kD);
    }

    /**
     * 
     */
    public void setTurnPID(double kP, double kI, double kD) {
        this.swerveModuleIO.setTurnPID(kP, kI, kD);
    }

    /**
     * 
     */
    public void updateInputs() {
        this.swerveModuleIO.updateInputs(this.inputs);

        Logger.processInputs("SwerveDrive/Module" + Integer.toString(index), inputs);
    }
}
