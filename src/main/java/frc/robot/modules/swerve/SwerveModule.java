package frc.robot.modules.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

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
        int sampleCount = inputs.odometryDrivePositionsRad.length; // All signals are sampled together
        this.odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i]
                    * (SwerveConstants.cotsTurnConstants.wheelDiameter / 2);
            Rotation2d angle = inputs.odometryTurnPositions[i];
            this.odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        this.driveDisconnectedAlert.set(!inputs.data.driveConnected());
        this.turnDisconnectedAlert.set(!inputs.data.turnConnected());
        this.turnEncoderDisconnectedAlert.set(!inputs.data.turnEncoderConnected());

        // SmartDashboard.putNumber("Mod " + index + " CANcoder",
        // this.cancoder.getDegrees());
        SmartDashboard.putNumber("Mod " + index + " Angle", getPosition().angle.getDegrees());
        SmartDashboard.putNumber("Mod " + index + " Velocity", getState().speedMetersPerSecond);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.data.turnPosition());

        // Apply setpoints
        double vRadPerSec = state.speedMetersPerSecond / (SwerveConstants.cotsTurnConstants.wheelDiameter / 2);
        swerveModuleIO.setDriveSetpoint(vRadPerSec);
        swerveModuleIO.setTurnSetpoint(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runDriveCharacterization(double output) {
        swerveModuleIO.setTurnSetpoint(Rotation2d.fromRotations(0.0));
        swerveModuleIO.setDriveVoltage(output);
    }

    /**
     * Runs the module angle with the specified voltage while not moving the drive
     * motor.
     */
    public void runTurnCharacterization(double volts) {
        swerveModuleIO.setTurnVoltage(volts);
        swerveModuleIO.setDriveVoltage(0.0);
    }

    /**
     * Runs the module
     */
    public void runVelocity(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(this.inputs.data.turnPosition());

        // Apply setpoints
        double volts = MathUtil.clamp((state.speedMetersPerSecond / SwerveConstants.maxLinearSpeed) * 12, -12.0, 12.0);

        swerveModuleIO.setDriveVoltage(volts);
        swerveModuleIO.setTurnSetpoint(state.angle);
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        swerveModuleIO.setTurnVoltage(0.0);
        swerveModuleIO.setDriveVoltage(0.0);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getAngle() {
        return this.inputs.data.turnPosition();
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
        return this.inputs.data.drivePositionRad() * (SwerveConstants.cotsTurnConstants.wheelDiameter / 2);
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return this.inputs.data.driveVelocityRadPerSec() * (SwerveConstants.cotsTurnConstants.wheelDiameter / 2);
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
        return this.inputs.data.drivePositionRad();
    }

    /**
     * Returns the module velocity in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(this.inputs.data.driveVelocityRadPerSec());
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
