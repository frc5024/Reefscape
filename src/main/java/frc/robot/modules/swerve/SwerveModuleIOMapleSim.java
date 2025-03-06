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

/**
 * Physics sim implementation of module IO.
 */
public class SwerveModuleIOMapleSim implements SwerveModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController = new PIDController(MapleSimConstants.driveSimP, 0,
            MapleSimConstants.driveSimD);
    private final PIDController turnController = new PIDController(MapleSimConstants.turnSimP, 0,
            MapleSimConstants.turnSimD);
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
                this.moduleSimulation.getSteerAbsoluteFacing(),
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
    public void setDriveSetpoint(double velocityRadPerSec) {
        this.driveClosedLoop = true;
        this.driveFFVolts = MapleSimConstants.driveSimKs * Math.signum(velocityRadPerSec)
                + MapleSimConstants.driveSimKv * velocityRadPerSec;
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