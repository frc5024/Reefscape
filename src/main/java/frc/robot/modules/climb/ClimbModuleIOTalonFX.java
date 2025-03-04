package frc.robot.modules.climb;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.PhoenixUtil;

/**
 * 
 */
public class ClimbModuleIOTalonFX implements ClimbModuleIO {
    public static final double reduction = 600.0;

    // Hardware
    private final TalonFX talonFX;

    // Status Signals
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Current> torqueCurrentAmps;
    private final StatusSignal<Temperature> temp;

    // Control Requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    // Connected debouncers
    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

    /**
     * 
     */
    public ClimbModuleIOTalonFX() {
        this.talonFX = new TalonFX(ClimbConstants.climbMotorID);

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Feedback.SensorToMechanismRatio = reduction;
        talonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
        talonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(245);
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        tryUntilOk(5, () -> this.talonFX.getConfigurator().apply(talonFXConfiguration, 0.25));
        tryUntilOk(5, () -> this.talonFX.setPosition(0.0));

        this.position = talonFX.getPosition();
        this.velocity = talonFX.getVelocity();
        this.appliedVolts = talonFX.getMotorVoltage();
        this.supplyCurrentAmps = talonFX.getSupplyCurrent();
        this.torqueCurrentAmps = talonFX.getTorqueCurrent();
        this.temp = talonFX.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, this.position, this.velocity, this.appliedVolts, this.supplyCurrentAmps, this.torqueCurrentAmps,
                this.temp);
        talonFX.optimizeBusUtilization();

        PhoenixUtil.registerSignals(
                false, this.position, this.velocity, this.appliedVolts, this.supplyCurrentAmps, this.torqueCurrentAmps,
                this.temp);
    }

    @Override
    public void updateInputs(ClimbModuleIOInputs inputs) {
        inputs.data = new ClimbModuleIOData(
                motorConnectedDebouncer.calculate(
                        BaseStatusSignal.isAllGood(
                                this.position, this.velocity, this.appliedVolts, this.supplyCurrentAmps, this.temp)),
                this.position.getValue().in(Radians),
                this.velocity.getValue().in(RadiansPerSecond),
                this.appliedVolts.getValueAsDouble(),
                this.torqueCurrentAmps.getValueAsDouble(),
                this.supplyCurrentAmps.getValueAsDouble(),
                this.temp.getValueAsDouble());
    }

    @Override
    public void runTorqueCurrent(double current) {
        this.talonFX.setControl(this.torqueCurrentRequest.withOutput(current));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(
                () -> {
                    tryUntilOk(
                            5,
                            () -> this.talonFX.setNeutralMode(
                                    enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25));
                })
                .start();
    }
}
