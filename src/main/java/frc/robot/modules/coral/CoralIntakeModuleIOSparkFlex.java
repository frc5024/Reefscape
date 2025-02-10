package frc.robot.modules.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
public class CoralIntakeModuleIOSparkFlex implements CoralIntakeModuleIO {
    /* Constants */
    private final int TOP_MOTOR_CHANNEL = 51;
    private final int BOTTOM_MOTOR_CHANNEL = 52;

    private final int LINEBREAK_CHANNEL = 0;

    private final int SERVO_CHANNEL = 0;

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

    private final double MOTOR_INTAKE_SPEED = -0.1;
    private final double MOTOR_EJECT_SPEED = 0.1;

    /* Hardware */
    private final SparkFlex topMotor;
    private final SparkFlex bottomMotor;
    private final DigitalInput lineBreak;

    /* Variables */
    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public CoralIntakeModuleIOSparkFlex() {
        this.topMotor = new SparkFlex(TOP_MOTOR_CHANNEL, SparkFlex.MotorType.kBrushless);
        this.bottomMotor = new SparkFlex(BOTTOM_MOTOR_CHANNEL, SparkFlex.MotorType.kBrushless);

        this.topMotor.configure(TOP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.bottomMotor.configure(BOTTOM_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.lineBreak = new DigitalInput(LINEBREAK_CHANNEL);
    }

    @Override
    public void updateInputs(CoralIntakeIOInputs inputs) {
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
    public boolean hasCoral() {
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
