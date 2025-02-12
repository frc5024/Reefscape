package frc.robot.modules.elevator;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class ElevatorModuleIOSim implements ElevatorModuleIO {
    // State given by elevator carriage position and velocity
    // Input given by torque current to motor
    private Vector<N2> simState;
    private double inputTorqueCurrent = 0.0;
    private double appliedVolts = 0.0;

    private final PIDController pidController;
    private boolean closedLoop = false;
    private double feedforward = 0.0;

    /**
     * 
     */
    public ElevatorModuleIOSim() {
        this.simState = VecBuilder.fill(0.0, 0.0);

        double[] elevatorPIDs = PIDConstants.getElevatorPIDs();
        this.pidController = new PIDController(elevatorPIDs[0], elevatorPIDs[1], elevatorPIDs[2]);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (!closedLoop) {
            this.pidController.reset();
            update(RobotConstants.LOOP_PERIOD_SECS);
        } else {
            // Run control at 1khz
            for (int i = 0; i < RobotConstants.LOOP_PERIOD_SECS / (1.0 / 1000.0); i++) {
                setInputTorqueCurrent(
                        this.pidController.calculate(simState.get(0) / ElevatorConstants.drumRadiusMeters)
                                + this.feedforward);
                update(1.0 / 1000.0);
            }
        }

        inputs.connected = true;
        inputs.positionRad = this.simState.get(0) / ElevatorConstants.drumRadiusMeters;
        inputs.velocityRadPerSec = this.simState.get(1) / ElevatorConstants.drumRadiusMeters;
        inputs.appliedVolts = new double[] { this.appliedVolts };
        inputs.currentAmps = new double[] { Math.copySign(this.inputTorqueCurrent, this.appliedVolts) };
    }

    @Override
    public void runOpenLoop(double output) {
        closedLoop = false;
        setInputTorqueCurrent(output);
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
        closedLoop = true;
        this.pidController.setSetpoint(positionRad);
        this.feedforward = feedforward;
    }

    @Override
    public void runVolts(double volts) {
        closedLoop = false;
        setInputVoltage(volts);
    }

    @Override
    public void stop() {
        runOpenLoop(0.0);
    }

    /**
     * 
     */
    private void setInputTorqueCurrent(double torqueCurrent) {
        this.inputTorqueCurrent = torqueCurrent;
        this.appliedVolts = ElevatorConstants.gearbox.getVoltage(
                ElevatorConstants.gearbox.getTorque(this.inputTorqueCurrent),
                simState.get(1, 0) / ElevatorConstants.drumRadiusMeters);
        this.appliedVolts = MathUtil.clamp(this.appliedVolts, -12.0, 12.0);
    }

    /**
     * 
     */
    private void setInputVoltage(double voltage) {
        setInputTorqueCurrent(
                ElevatorConstants.gearbox.getCurrent(this.simState.get(1) / ElevatorConstants.drumRadiusMeters,
                        voltage));
    }

    /**
     * 
     */
    private void update(double dt) {
        this.inputTorqueCurrent = MathUtil.clamp(
                this.inputTorqueCurrent, -ElevatorConstants.gearbox.stallCurrentAmps / 2.0,
                ElevatorConstants.gearbox.stallCurrentAmps / 2.0);

        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> ElevatorConstants.A.times(x)
                        .plus(ElevatorConstants.B.times(u))
                        .plus(VecBuilder.fill(0.0, -9.807 * ElevatorConstants.ANGLE.getSin())),
                this.simState,
                MatBuilder.fill(Nat.N1(), Nat.N1(), this.inputTorqueCurrent),
                dt);

        // Apply limits
        this.simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (this.simState.get(0) <= 0.0) {
            this.simState.set(1, 0, 0.0);
            this.simState.set(0, 0, 0.0);
        }

        if (this.simState.get(0) >= ElevatorConstants.HEIGHT_IN_METERS) {
            this.simState.set(1, 0, 0.0);
            this.simState.set(0, 0, ElevatorConstants.HEIGHT_IN_METERS);
        }
    }

    @Override
    public void updatePID(double kP, double kI, double kD) {
        this.pidController.setPID(kP, kI, kD);
    }
}
