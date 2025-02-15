package frc.robot.modules.swerve;

import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.CTREConfigs;
import frc.robot.utils.PhoenixOdometryThread;
import frc.robot.utils.SwerveModuleConstants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>
 * Device configuration and other behaviors not exposed by TunerConstants can be
 * customized here.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turnConfig;
    private final CTREConfigs ctreConfigs;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    private final Rotation2d angleOffset;

    /**
     * 
     */
    public SwerveModuleIOTalonFX(SwerveModuleConstants swerveModuleConstants) {
        this.ctreConfigs = new CTREConfigs();

        this.driveTalon = new TalonFX(swerveModuleConstants.driveMotorID, "rio");
        this.turnTalon = new TalonFX(swerveModuleConstants.angleMotorID, "rio");
        this.cancoder = new CANcoder(swerveModuleConstants.cancoderID, "rio");
        this.angleOffset = swerveModuleConstants.angleOffset;

        // Configure drive motor
        this.driveConfig = this.ctreConfigs.getDriveConfig();
        tryUntilOk(5, () -> this.driveTalon.getConfigurator().apply(this.driveConfig, 0.25));
        tryUntilOk(5, () -> this.driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        this.turnConfig = this.ctreConfigs.getAngleConfig();
        tryUntilOk(5, () -> this.turnTalon.getConfigurator().apply(this.turnConfig, 0.25));
        resetToAbsolute();

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = this.ctreConfigs.getCancoderConfig();
        this.cancoder.getConfigurator().apply(cancoderConfig);

        // Create timestamp queue
        this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        this.drivePosition = this.driveTalon.getPosition();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this.driveTalon.getPosition());
        this.driveVelocity = this.driveTalon.getVelocity();
        this.driveAppliedVolts = this.driveTalon.getMotorVoltage();
        this.driveCurrent = this.driveTalon.getStatorCurrent();

        // Create turn status signals
        this.turnAbsolutePosition = this.cancoder.getAbsolutePosition();
        this.turnPosition = this.turnTalon.getPosition();
        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this.turnTalon.getPosition());
        this.turnVelocity = this.turnTalon.getVelocity();
        this.turnAppliedVolts = this.turnTalon.getMotorVoltage();
        this.turnCurrent = this.turnTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(PhoenixOdometryThread.ODOMETRY_FREQUENCY, this.drivePosition,
                this.turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                this.driveVelocity,
                this.driveAppliedVolts,
                this.driveCurrent,
                this.turnAbsolutePosition,
                this.turnVelocity,
                this.turnAppliedVolts,
                this.turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(this.drivePosition, this.driveVelocity, this.driveAppliedVolts,
                this.driveCurrent);
        var turnStatus = BaseStatusSignal.refreshAll(this.turnPosition, this.turnVelocity, this.turnAppliedVolts,
                this.turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(this.turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = this.driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(this.drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(this.driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = this.driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = this.driveCurrent.getValueAsDouble();

        // Update turn inputs
        inputs.turnConnected = this.turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = this.turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(this.turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(this.turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(this.turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = this.turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = this.turnCurrent.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryTimestamps = this.timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = this.drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value))
                .toArray();
        inputs.odometryTurnPositions = this.turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);
        this.timestampQueue.clear();
        this.drivePositionQueue.clear();
        this.turnPositionQueue.clear();
    }

    /**
     * 
     */
    private Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(this.cancoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * 
     */
    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - this.angleOffset.getRotations();
        this.turnTalon.setPosition(absolutePosition);
    }

    @Override
    public void runDriveOpenLoop(double output) {
        this.driveTalon.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void runTurnOpenLoop(double output) {
        this.turnTalon.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        this.driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
    }

    @Override
    public void runTurnPosition(Rotation2d rotation) {
        this.turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    }

    @Override
    public void resetDrivePID() {
        this.driveTalon.getConfigurator().refresh(TunerConstants.driveGains);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        this.driveTalon.getConfigurator().refresh(slot0Configs);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        this.turnTalon.getConfigurator().refresh(slot0Configs);
    }
}
