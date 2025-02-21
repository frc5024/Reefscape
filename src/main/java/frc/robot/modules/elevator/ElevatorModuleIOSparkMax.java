package frc.robot.modules.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PIDConstants;

/**
 * 
 */
public class ElevatorModuleIOSparkMax implements ElevatorModuleIO {
    /* Constants */
    private final int LEFT_MOTOR_ID = 60;
    private final int RIGHT_MOTOR_ID = 61;
    private final int ZERO_LIMIT_CHANNEL_ID = 8;
    private final int STOPPING_LIMIT_CHANNEL_ID = 1;

    private final SparkBaseConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(35)
            .secondaryCurrentLimit(40)
            .inverted(true);
    private final SparkBaseConfig RIGHT_MOTOR_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(35)
            .secondaryCurrentLimit(40)
            .follow(LEFT_MOTOR_ID, false);

    private final double ENCODER_ZERO_POSITION = 0.0;

    /* Hardware */
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    /* Variables */
    private final PIDController pidController;
    private boolean closedLoop = false;
    private double feedforward = 0.145;

    private double appliedVoltage = 0.0;

    /**
     * 
     */
    public ElevatorModuleIOSparkMax() {
        this.leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        this.rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

        this.leftMotor.configure(LEFT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.rightMotor.configure(RIGHT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.topLimitSwitch = new DigitalInput(ZERO_LIMIT_CHANNEL_ID);
        this.bottomLimitSwitch = new DigitalInput(STOPPING_LIMIT_CHANNEL_ID);

        double[] elevatorPIDs = PIDConstants.getElevatorPIDs();
        this.pidController = new PIDController(elevatorPIDs[0], elevatorPIDs[1], elevatorPIDs[2]);

        // reset the encoder
        this.leftMotor.getEncoder().setPosition(ENCODER_ZERO_POSITION);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (this.closedLoop) {
            double output = this.pidController.calculate(this.leftMotor.getEncoder().getPosition() + this.feedforward);
            this.runPosition(output, this.feedforward);
        }

        inputs.connected = true;
        inputs.positionRad = this.leftMotor.getEncoder().getPosition();
        inputs.velocityRadPerSec = this.leftMotor.getEncoder().getVelocity();
        inputs.appliedVolts = new double[] { this.leftMotor.getBusVoltage(), this.leftMotor.getAppliedOutput() };
        inputs.currentAmps = new double[] { this.leftMotor.getOutputCurrent(), this.rightMotor.getOutputCurrent() };
        inputs.tempCelsius = new double[] { this.leftMotor.getMotorTemperature(),
                this.rightMotor.getMotorTemperature() };
    }

    @Override
    public boolean isAtBottom() {
        return !this.bottomLimitSwitch.get();
    }

    @Override
    public boolean isAtTop() {
        return !this.topLimitSwitch.get();
    }

    @Override
    public void runOpenLoop(double output) {
        this.closedLoop = false;
        this.appliedVoltage = MathUtil.clamp(output * 12, -12.0, 12.0);
        this.leftMotor.setVoltage(this.appliedVoltage);
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
        this.closedLoop = true;
        this.pidController.setSetpoint(positionRad);
        this.feedforward = feedforward;
    }

    @Override
    public void stop() {
        this.appliedVoltage = 0.0;
        this.leftMotor.setVoltage(this.appliedVoltage);
    }

    @Override
    public void updatePID(double kP, double kI, double kD) {
        this.pidController.setPID(kP, kI, kD);
    }

    /**
     * 
     */
    public void zeroEncoder() {
        this.leftMotor.getEncoder().setPosition(ENCODER_ZERO_POSITION);
    }
}
