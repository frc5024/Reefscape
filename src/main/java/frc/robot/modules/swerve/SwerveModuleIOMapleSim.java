package frc.robot.modules.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.MapleSimConstants;
import frc.robot.Constants.PIDConstants;

/**
 * Physics sim implementation of module IO.
 */
public class SwerveModuleIOMapleSim implements SwerveModuleIO {
    private static final double[] DRIVE_PIDs = PIDConstants.getDrivePIDs();
    private static final double[] TURN_PIDs = PIDConstants.getTurnPIDs();

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController = new PIDController(DRIVE_PIDs[0], DRIVE_PIDs[1], DRIVE_PIDs[2]);
    private final PIDController turnController = new PIDController(TURN_PIDs[0], TURN_PIDs[1], TURN_PIDs[2]);

    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    /**
     * 
     */
    public SwerveModuleIOMapleSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(MapleSimConstants.driveMotorCurrentLimit));
        this.turnMotor = moduleSimulation.useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(MapleSimConstants.turnMotorCurrentLimit));

        // Enable wrapping for turn PID
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts
                    + this.driveController.calculate(
                            this.moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            this.driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = this.turnController.calculate(
                    this.moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            this.turnController.reset();
        }

        // Update simulation state
        this.driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        this.turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

        // Update drive inputs
        inputs.data = new SwerveModuleIOData(
                true,
                this.moduleSimulation.getDriveWheelFinalPosition().in(Radians),
                this.moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
                this.driveAppliedVolts,
                Math.abs(this.moduleSimulation.getDriveMotorSupplyCurrent().in(Amps)),
                0.0,
                true,
                true,
                this.moduleSimulation.getSteerAbsoluteFacing(),
                this.moduleSimulation.getSteerAbsoluteFacing().getDegrees(),
                this.moduleSimulation.getSteerAbsoluteFacing(),
                this.moduleSimulation.getSteerAbsoluteFacing().getDegrees(),
                this.moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond),
                this.turnAppliedVolts,
                Math.abs(this.moduleSimulation.getSteerMotorSupplyCurrent().in(Amps)),
                0.0);

        // Update odometry inputs
        inputs.odometryDrivePositionsRad = Arrays.stream(this.moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        inputs.odometryTurnPositions = this.moduleSimulation.getCachedSteerAbsolutePositions();
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
    public void setDriveSetpoint(double velocityRadPerSec) {
        this.driveClosedLoop = true;
        this.driveFFVolts = DRIVE_PIDs[3] * Math.signum(velocityRadPerSec) + DRIVE_PIDs[4] * velocityRadPerSec;
        this.driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setDriveVoltage(double volts) {
        this.driveClosedLoop = false;
        this.driveAppliedVolts = volts;
    }

    @Override
    public void setTurnSetpoint(Rotation2d rotation) {
        this.turnClosedLoop = true;
        this.turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setTurnVoltage(double volts) {
        this.turnClosedLoop = false;
        this.turnAppliedVolts = volts;
    }
}