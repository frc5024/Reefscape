package frc.robot.modules.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.SparkUtil;

/**
 * 
 */
public class AlgaeModuleIOSparkMax implements AlgaeModuleIO {
    private final int TOP_MOTOR_ID = 3;
    private final int BOTTOM_MOTOR_ID = 62;
    private final int LINEBREAK_CHANNEL = 9;
    private final double MOTOR_INTAKE_SPEED = -0.5;
    private final double MOTOR_EJECT_SPEED = 0.5;

    private final SparkBaseConfig TOP_MOTOR_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(35)
            .secondaryCurrentLimit(40)
            .inverted(false);
    private final SparkBaseConfig BOTTOM_MOTOR_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(35)
            .secondaryCurrentLimit(40)
            .inverted(true);

    private final Debouncer connectedDebouncer = new Debouncer(.5);

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final DigitalInput lineBreak;

    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public AlgaeModuleIOSparkMax() {
        this.topMotor = new SparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
        this.bottomMotor = new SparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);

        this.topMotor.configure(TOP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.bottomMotor.configure(BOTTOM_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.lineBreak = new DigitalInput(LINEBREAK_CHANNEL);
    }

    @Override
    public void updateInputs(AlgaeModuleIOInputs inputs) {
        SparkUtil.sparkStickyFault = false;
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.data = new AlgaeModuleIOData(
                this.connectedDebouncer.calculate(!SparkUtil.sparkStickyFault),
                this.topMotor.getEncoder().getPosition(),
                this.topMotor.getEncoder().getVelocity(),
                this.appliedVoltage,
                0.0,
                this.topMotor.getAppliedOutput(),
                this.topMotor.getMotorTemperature());
    }

    @Override
    public void eject() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_EJECT_SPEED * 12, -12.0, 12.0);
        this.topMotor.setVoltage(this.appliedVoltage);
        this.bottomMotor.setVoltage(this.appliedVoltage);
    }

    @Override
    public boolean hasAlgae() {
        return this.lineBreak.get();
    }

    @Override
    public void intake() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_INTAKE_SPEED * 12, -12.0, 12.0);
        this.topMotor.setVoltage(this.appliedVoltage);
        this.bottomMotor.setVoltage(this.appliedVoltage);
    }

    @Override
    public void stop() {
        this.appliedVoltage = 0.0;
        this.topMotor.setVoltage(this.appliedVoltage);
        this.bottomMotor.setVoltage(this.appliedVoltage);
    }
}
