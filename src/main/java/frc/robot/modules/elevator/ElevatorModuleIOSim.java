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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class ElevatorModuleIOSim implements ElevatorModuleIO {
    private final double drumRadiusMeters = Units.inchesToMeters(6.0);
    private final double reduction = 5.0;
    private final double carriageMassKg = Units.lbsToKilograms(6.0);
    private final double stagesMassKg = Units.lbsToKilograms(12.0);
    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(2).withReduction(reduction);

    private final Matrix<N2, N2> A = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp
                    / (gearbox.rOhms
                            * Math.pow(drumRadiusMeters, 2)
                            * (carriageMassKg + stagesMassKg)
                            * gearbox.KvRadPerSecPerVolt));

    private final Vector<N2> B = VecBuilder.fill(
            0.0, gearbox.KtNMPerAmp / (drumRadiusMeters * (carriageMassKg + stagesMassKg)));

    // State given by elevator carriage position and velocity
    // Input given by torque current to motor
    private Vector<N2> simState;
    private double inputTorqueCurrent = 0.0;
    private double appliedVolts = 0.0;

    private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
    private boolean closedLoop = false;
    private double feedforward = 0.0;

    /* Mechanisim2d Display for Monitoring the Elevator Position */
    private final ElevatorMechanism elevatorMechanism = new ElevatorMechanism();

    /**
     * 
     */
    public ElevatorModuleIOSim() {
        simState = VecBuilder.fill(0.0, 0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (!closedLoop) {
            controller.reset();
            update(RobotConstants.LOOP_PERIOD_SECS);
        } else {
            // Run control at 1khz
            for (int i = 0; i < RobotConstants.LOOP_PERIOD_SECS / (1.0 / 1000.0); i++) {
                setInputTorqueCurrent(
                        controller.calculate(simState.get(0) / drumRadiusMeters) + feedforward);
                update(1.0 / 1000.0);
            }
        }

        inputs.positionRad = simState.get(0) / drumRadiusMeters;
        inputs.velocityRadPerSec = simState.get(1) / drumRadiusMeters;
        inputs.appliedVolts = new double[] { appliedVolts };
        inputs.currentAmps = new double[] { Math.copySign(inputTorqueCurrent, appliedVolts) };

        this.elevatorMechanism.update();
    }

    @Override
    public void runOpenLoop(double output) {
        closedLoop = false;
        setInputTorqueCurrent(output);
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

    @Override
    public void runPosition(double positionRad, double feedforward) {
        closedLoop = true;
        controller.setSetpoint(positionRad);
        this.feedforward = feedforward;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    /**
     * 
     */
    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts = gearbox.getVoltage(
                gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0) / drumRadiusMeters);
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    /**
     * 
     */
    private void setInputVoltage(double voltage) {
        setInputTorqueCurrent(gearbox.getCurrent(simState.get(1) / drumRadiusMeters, voltage));
    }

    /**
     * 
     */
    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(
                inputTorqueCurrent, -gearbox.stallCurrentAmps / 2.0, gearbox.stallCurrentAmps / 2.0);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x)
                        .plus(B.times(u))
                        .plus(
                                VecBuilder.fill(
                                        0.0,
                                        -9.807
                                                * ElevatorConstants.ANGLE.getSin())),
                simState,
                MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
                dt);
        // Apply limits
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (simState.get(0) <= 0.0) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, 0.0);
        }
        if (simState.get(0) >= ElevatorConstants.HEIGHT_IN_METERS) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, ElevatorConstants.HEIGHT_IN_METERS);
        }
    }
}
