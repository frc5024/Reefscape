package frc.robot.modules.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import frc.robot.Constants.PIDConstants;
import frc.robot.utils.PhoenixOdometryThread;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.SwerveModuleBuilder;

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

    // Voltage control requests
    private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(false);
    private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(false);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(false);
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrentAmps;
    private final StatusSignal<Current> driveTorqueCurrentAmps;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnSupplyCurrentAmps;
    private final StatusSignal<Current> turnTorqueCurrentAmps;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    private final Rotation2d encoderOffset;

    /**
     * 
     */
    public SwerveModuleIOTalonFX(SwerveModuleBuilder swerveModuleBuilder) {
        this.driveTalon = new TalonFX(swerveModuleBuilder.driveMotorId, "rio");
        this.turnTalon = new TalonFX(swerveModuleBuilder.turnMotorId, "rio");
        this.cancoder = new CANcoder(swerveModuleBuilder.encoderChannel, "rio");
        this.encoderOffset = swerveModuleBuilder.encoderOffset;

        // Configure drive motor
        this.driveConfig = swerveModuleBuilder.getDriveConfig();
        tryUntilOk(5, () -> this.driveTalon.getConfigurator().apply(this.driveConfig, 0.25));
        tryUntilOk(5, () -> this.driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        this.turnConfig = swerveModuleBuilder.getTurnConfig();
        tryUntilOk(5, () -> this.turnTalon.getConfigurator().apply(this.turnConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = swerveModuleBuilder.getCancoderConfig();
        tryUntilOk(5, () -> this.cancoder.getConfigurator().apply(cancoderConfig));

        // Create drive status signals
        this.drivePosition = this.driveTalon.getPosition();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this.driveTalon.getPosition());
        this.driveVelocity = this.driveTalon.getVelocity();
        this.driveAppliedVolts = this.driveTalon.getMotorVoltage();
        this.driveSupplyCurrentAmps = this.driveTalon.getSupplyCurrent();
        this.driveTorqueCurrentAmps = this.driveTalon.getTorqueCurrent();

        // Create turn status signals
        this.turnAbsolutePosition = this.cancoder.getAbsolutePosition();
        this.turnPosition = this.turnTalon.getPosition();
        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(this.turnTalon.getPosition());
        this.turnVelocity = this.turnTalon.getVelocity();
        this.turnAppliedVolts = this.turnTalon.getMotorVoltage();
        this.turnSupplyCurrentAmps = this.turnTalon.getSupplyCurrent();
        this.turnTorqueCurrentAmps = this.turnTalon.getTorqueCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(PhoenixOdometryThread.ODOMETRY_FREQUENCY, this.drivePosition,
                this.turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                this.driveVelocity,
                this.driveAppliedVolts,
                this.driveSupplyCurrentAmps,
                this.driveTorqueCurrentAmps,
                this.turnAbsolutePosition,
                this.turnVelocity,
                this.turnAppliedVolts,
                this.turnSupplyCurrentAmps,
                this.turnTorqueCurrentAmps);
        tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(this.driveTalon, this.turnTalon, this.cancoder));

        // Register signals for refresh
        PhoenixUtil.registerSignals(
                false,
                this.drivePosition,
                this.driveVelocity,
                this.driveAppliedVolts,
                this.driveSupplyCurrentAmps,
                this.driveTorqueCurrentAmps,
                this.turnPosition,
                this.turnAbsolutePosition,
                this.turnVelocity,
                this.turnAppliedVolts,
                this.turnSupplyCurrentAmps,
                this.turnTorqueCurrentAmps);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.data = new SwerveModuleIOData(
                this.driveConnectedDebounce.calculate(BaseStatusSignal.isAllGood(
                        this.drivePosition,
                        this.driveVelocity,
                        this.driveAppliedVolts,
                        this.driveSupplyCurrentAmps,
                        this.driveTorqueCurrentAmps)),
                Units.rotationsToRadians(this.drivePosition.getValueAsDouble()),
                Units.rotationsToRadians(this.driveVelocity.getValueAsDouble()),
                this.driveAppliedVolts.getValueAsDouble(),
                this.driveSupplyCurrentAmps.getValueAsDouble(),
                this.driveTorqueCurrentAmps.getValueAsDouble(),
                this.turnConnectedDebounce.calculate(BaseStatusSignal.isAllGood(
                        this.turnPosition,
                        this.turnVelocity,
                        this.turnAppliedVolts,
                        this.turnSupplyCurrentAmps,
                        this.turnTorqueCurrentAmps)),
                this.turnEncoderConnectedDebounce.calculate(BaseStatusSignal.isAllGood(this.turnAbsolutePosition)),
                Rotation2d.fromRotations(this.turnAbsolutePosition.getValueAsDouble()).plus(this.encoderOffset),
                this.turnAbsolutePosition.getValue().in(Degrees),
                Rotation2d.fromRotations(this.turnPosition.getValueAsDouble()),
                this.turnPosition.getValue().in(Degrees),
                Units.rotationsToRadians(this.turnVelocity.getValueAsDouble()),
                this.turnAppliedVolts.getValueAsDouble(),
                this.turnSupplyCurrentAmps.getValueAsDouble(),
                this.turnTorqueCurrentAmps.getValueAsDouble());

        // Update odometry inputs
        inputs.odometryDrivePositionsRad = this.drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians).toArray();
        inputs.odometryTurnPositions = this.turnPositionQueue.stream()
                .map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        this.drivePositionQueue.clear();
        this.turnPositionQueue.clear();
    }

    @Override
    public void resetDrivePID() {
        double[] drivePIDs = PIDConstants.getDrivePIDs();
        tryUntilOk(5, () -> this.driveTalon.getConfigurator()
                .refresh(new Slot0Configs().withKP(drivePIDs[0]).withKI(drivePIDs[1]).withKD(drivePIDs[2])));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        tryUntilOk(5, () -> this.driveTalon.getConfigurator()
                .refresh(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD)));
    }

    @Override
    public void setDriveSetpoint(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        this.driveTalon.setControl(this.motionMagicVelocityVoltage.withVelocity(velocityRotPerSec));
    }

    @Override
    public void setDriveVoltage(double volts) {
        this.driveTalon.setControl(this.driveVoltage.withOutput(volts));
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        tryUntilOk(5, () -> this.turnTalon.getConfigurator()
                .refresh(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD)));
    }

    @Override
    public void setTurnSetpoint(Rotation2d rotation) {
        this.turnTalon.setControl(this.motionMagicVoltage.withPosition(rotation.getRotations()));
    }

    @Override
    public void setTurnVoltage(double volts) {
        this.turnTalon.setControl(this.turnVoltage.withOutput(volts));
    }
}
