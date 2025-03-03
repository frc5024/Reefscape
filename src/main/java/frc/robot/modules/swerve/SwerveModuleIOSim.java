package frc.robot.modules.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
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

    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveMotorSim;
    private final DCMotorSim turnMotorSim;

    private PIDController driveController = new PIDController(DRIVE_PIDs[0], DRIVE_PIDs[1], DRIVE_PIDs[2]);
    private PIDController turnController = new PIDController(TURN_PIDs[0], TURN_PIDs[1], TURN_PIDs[2]);
    private SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(DRIVE_PIDs[3], DRIVE_PIDs[4],
            DRIVE_PIDs[5]);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
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
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (this.driveClosedLoop) {
            this.driveAppliedVolts = driveController.calculate(this.driveMotorSim.getAngularVelocityRadPerSec())
                    + this.simpleMotorFeedforward.calculate(this.driveMotorSim.getAngularVelocityRadPerSec());
        } else {
            this.driveController.reset();
        }
        if (this.turnClosedLoop) {
            this.turnAppliedVolts = turnController.calculate(turnMotorSim.getAngularPositionRad());
        } else {
            this.turnController.reset();
        }

        // Update simulation state
        this.driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -16.0, 16.0));
        this.turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -16.0, 16.0));
        this.driveMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);
        this.turnMotorSim.update(RobotConstants.LOOP_PERIOD_SECS);

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
        this.driveClosedLoop = false;
        this.driveAppliedVolts = output;
    }

    @Override
    public void runTurnOpenLoop(double output) {
        this.turnClosedLoop = false;
        this.turnAppliedVolts = output;
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec) {
        this.driveClosedLoop = true;
        this.driveController.setSetpoint(velocityRadPerSec);
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
