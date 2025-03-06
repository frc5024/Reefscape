package frc.robot.modules.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;

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

    private static final double DRIVE_KS = DRIVE_PIDs[3]; // 0.00865;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = DRIVE_PIDs[4]; // 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

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
        this.driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, constants.DriveInertia,
                        constants.DriveMotorGearRatio),
                DRIVE_GEARBOX);

        this.turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_GEARBOX);

        // Enable wrapping for turn PID
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (this.driveClosedLoop) {
            this.driveAppliedVolts = driveFFVolts
                    + driveController.calculate(this.driveMotorSim.getAngularVelocityRadPerSec());
        } else {
            this.driveController.reset();
        }
        if (this.turnClosedLoop) {
            this.turnAppliedVolts = this.turnController.calculate(turnMotorSim.getAngularPositionRad());
        } else {
            this.turnController.reset();
        }

        // Update simulation state
        this.driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        this.turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        this.driveMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);
        this.turnMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);

        // Update drive inputs
        inputs.data = new SwerveModuleIOData(
                true,
                this.driveMotorSim.getAngularPositionRad(),
                this.driveMotorSim.getAngularVelocityRadPerSec(),
                this.driveAppliedVolts,
                Math.abs(this.driveMotorSim.getCurrentDrawAmps()),
                0.0,
                true,
                true,
                new Rotation2d(this.turnMotorSim.getAngularPositionRad()),
                new Rotation2d(this.turnMotorSim.getAngularPositionRad()),
                this.turnMotorSim.getAngularVelocityRadPerSec(),
                this.turnAppliedVolts,
                Math.abs(this.turnMotorSim.getCurrentDrawAmps()),
                0.0);

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryDrivePositionsRad = new double[] { inputs.data.drivePositionRad() };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.data.turnPosition() };
    }

    @Override
    public void setDriveVoltage(double output) {
        this.driveClosedLoop = false;
        this.driveAppliedVolts = output;
    }

    @Override
    public void setTurnVoltage(double output) {
        this.turnClosedLoop = false;
        this.turnAppliedVolts = output;
    }

    @Override
    public void setDriveSetpoint(double velocityRadPerSec) {
        this.driveClosedLoop = true;
        this.driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
        this.driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnSetpoint(Rotation2d rotation) {
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
