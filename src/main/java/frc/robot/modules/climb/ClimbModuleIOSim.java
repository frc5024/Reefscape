package frc.robot.modules.climb;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.RobotConstants;

/**
 * 
 */
public class ClimbModuleIOSim implements ClimbModuleIO {
    private static final double moi = 0.0;
    private static final double cgRadius = 0.2;
    private static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1).withReduction(ClimbModuleIOTalonFX.reduction);
    private static final Matrix<N2, N2> A = MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
    private static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

    // Climber sim
    private Vector<N2> simState;
    private double inputTorqueCurrent = 0.0;
    private double appliedVolts = 0.0;

    /**
     * 
     */
    public ClimbModuleIOSim() {
        simState = VecBuilder.fill(Math.PI / 2.0, 0.0);
    }

    @Override
    public void updateInputs(ClimbModuleIOInputs inputs) {
        update(RobotConstants.LOOP_PERIOD_SECS);
        // SuperstructureVisualizer.updateSimIntake(simState.get(0));

        inputs.data = new ClimbModuleIOData(
                true,
                simState.get(0),
                simState.get(1),
                appliedVolts,
                Math.copySign(inputTorqueCurrent, appliedVolts),
                Math.copySign(inputTorqueCurrent, appliedVolts),
                0.0);
    }

    @Override
    public void runTorqueCurrent(double current) {
        inputTorqueCurrent = current;
        appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x)
                        .plus(B.times(u))
                        .plus(
                                -9.807
                                        * cgRadius
                                        * Rotation2d.fromRadians(simState.get(0)).getCos()
                                        / moi),
                simState,
                VecBuilder.fill(inputTorqueCurrent * 15), // Magic constant of doom
                dt);
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    }
}
