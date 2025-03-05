package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PIDConstants;

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

    private CANcoderConfiguration canCoderConfig;
    private TalonFXConfiguration driveTalonFXConfig;
    private TalonFXConfiguration turnTalonFXConfig;

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

        setDriveConfig();
        setTurnConfig();
        setCancoderConfig();
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
        this.canCoderConfig = new CANcoderConfiguration();

        this.canCoderConfig.MagnetSensor.MagnetOffset = 0.0; // this.encoderOffset.getRotations();
        this.canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // this.encoderInverted
        // ? SensorDirectionValue.Clockwise_Positive
        // : SensorDirectionValue.CounterClockwise_Positive;
    }

    /**
     * 
     */
    private void setDriveConfig() {
        this.driveTalonFXConfig = new TalonFXConfiguration();

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        this.driveTalonFXConfig.MotorOutput.Inverted = this.cotsDriveConstants.driveMotorInvert;
        this.driveTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Gear Ratio Config */
        this.driveTalonFXConfig.Feedback.SensorToMechanismRatio = this.cotsDriveConstants.driveGearRatio;

        /* Current Limiting */
        this.driveTalonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        this.driveTalonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        this.driveTalonFXConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        this.driveTalonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* PID Config */
        double[] drivePIDs = PIDConstants.getDrivePIDs();
        this.driveTalonFXConfig.Slot0.kP = drivePIDs[0];
        this.driveTalonFXConfig.Slot0.kI = drivePIDs[1];
        this.driveTalonFXConfig.Slot0.kD = drivePIDs[2];

        this.driveTalonFXConfig.Slot0.kS = drivePIDs[3];
        this.driveTalonFXConfig.Slot0.kV = drivePIDs[4];
        this.driveTalonFXConfig.Slot0.kA = drivePIDs[5];

        /* Open and Closed Loop Ramping */
        this.driveTalonFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        this.driveTalonFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        this.driveTalonFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
        this.driveTalonFXConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        this.driveTalonFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

        MotionMagicConfigs magicMagicConfigs = this.driveTalonFXConfig.MotionMagic;
        // magicMagicConfigs.MotionMagicCruiseVelocity = 80;
        magicMagicConfigs.MotionMagicAcceleration = 400;
        magicMagicConfigs.MotionMagicJerk = 4000;
    }

    /**
     * 
     */
    private void setTurnConfig() {
        this.turnTalonFXConfig = new TalonFXConfiguration();

        /** Swerve Turn Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        this.turnTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // this.turnTalonFXConfig.MotorOutput.Inverted = this.turnInverted
        // ? InvertedValue.Clockwise_Positive
        // : InvertedValue.CounterClockwise_Positive;
        this.turnTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Gear Ratio and Wrapping Config */
        this.turnTalonFXConfig.Feedback.SensorToMechanismRatio = this.cotsTurnConstants.angleGearRatio;
        this.turnTalonFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        this.turnTalonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        this.turnTalonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
        this.turnTalonFXConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        this.turnTalonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* PID Config */
        // this.turnTalonFXConfig.Slot0.kP = this.cotsTurnConstants.angleKP;
        // this.turnTalonFXConfig.Slot0.kI = this.cotsTurnConstants.angleKI;
        // this.turnTalonFXConfig.Slot0.kD = this.cotsTurnConstants.angleKD;

        double[] turnPIDs = PIDConstants.getTurnPIDs();
        this.turnTalonFXConfig.Slot0.kP = turnPIDs[0];
        this.turnTalonFXConfig.Slot0.kI = turnPIDs[1];
        this.turnTalonFXConfig.Slot0.kD = turnPIDs[2];

        this.turnTalonFXConfig.Slot0.kS = turnPIDs[3];
        this.turnTalonFXConfig.Slot0.kV = turnPIDs[4];
        this.turnTalonFXConfig.Slot0.kA = turnPIDs[5];
    }
}
