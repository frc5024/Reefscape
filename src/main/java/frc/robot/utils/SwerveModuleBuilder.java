package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 
 */
public class SwerveModuleBuilder {
    public final int driveMotorId;
    public final int turnMotorId;
    public final int encoderChannel;
    public final Rotation2d encoderOffset;

    private final boolean turnInverted;
    private final boolean encoderInverted;
    private final COTSTalonFXSwerveConstants cotsDriveConstants;
    private final COTSTalonFXSwerveConstants cotsTurnConstants;

    private final CANcoderConfiguration canCoderConfig;
    private final TalonFXConfiguration driveTalonFXConfig;
    private final TalonFXConfiguration turnTalonFXConfig;

    /**
     * 
     */
    public SwerveModuleBuilder(int driveMotorId, int turnMotorId, int encoderChannel, Rotation2d encoderOffset,
            boolean turnInverted, boolean encoderInverted, COTSTalonFXSwerveConstants cotsDriveConstants,
            COTSTalonFXSwerveConstants cotsTurnConstants) {
        this.driveMotorId = driveMotorId;
        this.turnMotorId = turnMotorId;
        this.encoderChannel = encoderChannel;
        this.encoderOffset = encoderOffset;
        this.turnInverted = turnInverted;
        this.encoderInverted = encoderInverted;
        this.cotsDriveConstants = cotsDriveConstants;
        this.cotsTurnConstants = cotsTurnConstants;

        this.canCoderConfig = new CANcoderConfiguration();
        this.driveTalonFXConfig = new TalonFXConfiguration();
        this.turnTalonFXConfig = new TalonFXConfiguration();

        setCancoderConfig();
        setDriveConfig();
        setTurnConfig();
    }

    /**
     * 
     */
    public CANcoderConfiguration getCancoderConfig() {
        return this.canCoderConfig;
    }

    public TalonFXConfiguration getDriveConfig() {
        return this.driveTalonFXConfig;
    }

    public TalonFXConfiguration getTurnConfig() {
        return this.turnTalonFXConfig;
    }

    /**
     * 
     */
    private void setCancoderConfig() {
        this.canCoderConfig.MagnetSensor.MagnetOffset = this.encoderOffset.getRotations();
        this.canCoderConfig.MagnetSensor.SensorDirection = this.cotsTurnConstants.cancoderInvert;
    }

    /**
     * 
     */
    private void setDriveConfig() {
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        this.driveTalonFXConfig.MotorOutput.Inverted = this.cotsDriveConstants.driveMotorInvert;
        this.driveTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Gear Ratio Config */
        this.driveTalonFXConfig.Feedback.SensorToMechanismRatio = this.cotsDriveConstants.driveGearRatio;

        /* Current Limiting */
        this.driveTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.driveTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
        // this.driveTalonFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        // this.driveTalonFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        this.driveTalonFXConfig.Slot0.kP = 0.112;
        this.driveTalonFXConfig.Slot0.kI = 0.0;
        this.driveTalonFXConfig.Slot0.kD = 0.0;
        this.driveTalonFXConfig.Slot0.kV = 0.0;

        /* Open and Closed Loop Ramping */
        this.driveTalonFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        this.driveTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        this.driveTalonFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
        this.driveTalonFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    }

    /**
     * 
     */
    private void setTurnConfig() {
        /** Swerve Turn Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        this.turnTalonFXConfig.MotorOutput.Inverted = this.cotsTurnConstants.angleMotorInvert;
        this.turnTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Gear Ratio and Wrapping Config */
        this.turnTalonFXConfig.Feedback.SensorToMechanismRatio = this.cotsTurnConstants.angleGearRatio;
        this.turnTalonFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        this.turnTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        this.turnTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
        // this.turnTalonFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
        // this.turnTalonFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        /* PID Config */
        this.turnTalonFXConfig.Slot0.kP = this.cotsTurnConstants.angleKP;
        this.turnTalonFXConfig.Slot0.kI = this.cotsTurnConstants.angleKI;
        this.turnTalonFXConfig.Slot0.kD = this.cotsTurnConstants.angleKD;
    }
}
