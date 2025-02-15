package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.ConstantsMiniBot.Swerve;

/**
 * 
 */
public final class CTREConfigs {
    private final TalonFXConfiguration swerveAngleFXConfig;
    private final CANcoderConfiguration swerveCANcoderConfig;
    private final TalonFXConfiguration swerveDriveFXConfig;

    public CTREConfigs() {
        this.swerveAngleFXConfig = new TalonFXConfiguration();
        this.swerveCANcoderConfig = new CANcoderConfiguration();
        this.swerveDriveFXConfig = new TalonFXConfiguration();

        setAngleConfig();
        setCancoderConfig();
        setDriveConfig();
    }

    /**
     * 
     */
    public TalonFXConfiguration getAngleConfig() {
        return this.swerveAngleFXConfig;
    }

    /**
     * 
     */
    public CANcoderConfiguration getCancoderConfig() {
        return this.swerveCANcoderConfig;
    }

    /**
     * 
     */
    public TalonFXConfiguration getDriveConfig() {
        return this.swerveDriveFXConfig;
    }

    /**
     * 
     */
    private void setAngleConfig() {
        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Swerve.angleCurrentLimit;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold =
        // Swerve.angleCurrentThreshold;
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold =
        // Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Swerve.angleKD;
    }

    /**
     * 
     */
    private void setCancoderConfig() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Swerve.cancoderInvert;
    }

    /**
     * 
     */
    private void setDriveConfig() {
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Swerve.driveCurrentLimit;
        // swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold =
        // Swerve.driveCurrentThreshold;
        // swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold =
        // Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kV = Swerve.driveKV;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Swerve.closedLoopRamp;
    }
}