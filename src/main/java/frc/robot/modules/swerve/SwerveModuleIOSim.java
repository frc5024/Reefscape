package frc.robot.modules.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.PIDConstants;

/**
 * Physics sim implementation of module IO. The sim models are configured using
 * a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double[] DRIVE_PIDs = PIDConstants.getDrivePIDs();
    private static final double[] TURN_PIDs = PIDConstants.getTurnPIDs();

    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveMotorSim;
    private final DCMotorSim turnMotorSim;

    private PIDController driveController = new PIDController(DRIVE_PIDs[0], DRIVE_PIDs[1], DRIVE_PIDs[2]);
    private PIDController turnController = new PIDController(TURN_PIDs[0], TURN_PIDs[1], TURN_PIDs[2]);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * 
     */
    public SwerveModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        // Create drive and turn sim models
        driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, constants.DriveInertia,
                        constants.DriveMotorGearRatio),
                DRIVE_GEARBOX);

        turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_GEARBOX);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveMotorSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnMotorSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveMotorSim.update(0.02);
        turnMotorSim.update(0.02);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveMotorSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveMotorSim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnMotorSim.getAngularPositionRad());
        inputs.turnPosition = new Rotation2d(turnMotorSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnMotorSim.getCurrentDrawAmps());

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
    }

    @Override
    public void runDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void runTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void runTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void resetDrivePID() {
        this.driveController.setPID(DRIVE_PIDs[0], DRIVE_PIDs[1], DRIVE_PIDs[2]);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        this.driveController.setPID(kP, kI, kD);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        this.turnController.setPID(kP, kI, kD);
    }
}
