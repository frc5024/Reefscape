package frc.robot.modules.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class AlgaeintakeModuleIOSparkMax implements AlgaeIntakeModuleIO {
    private final int TOP_MOTOR_ID = 3;
    private final int BOTTOM_MOTOR_ID = 62;
    private final int LINEBREAK_CHANNEL = 8;

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

    private final double MOTOR_INTAKE_SPEED = 0.8;
    private final double MOTOR_EJECT_SPEED = -0.6;

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final DigitalInput limitSwitch;

    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public AlgaeintakeModuleIOSparkMax() {
        this.topMotor = new SparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
        this.bottomMotor = new SparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);

        this.topMotor.configure(TOP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.bottomMotor.configure(BOTTOM_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.limitSwitch = new DigitalInput(LINEBREAK_CHANNEL);
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        inputs.connected = true;
        inputs.appliedVoltage = this.appliedVoltage;
        inputs.supplyCurrentAmps = this.topMotor.getAppliedOutput();
        inputs.tempCelsius = this.topMotor.getMotorTemperature();
    }

    @Override
    public void eject() {
        this.appliedVoltage = MathUtil.clamp(MOTOR_EJECT_SPEED * 12, -12.0, 12.0);
        this.topMotor.setVoltage(this.appliedVoltage);
        this.bottomMotor.setVoltage(this.appliedVoltage);
    }

    @Override
    public boolean hasAlgae() {
        return this.limitSwitch.get();
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
