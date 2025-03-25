package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.modules.climb.ClimbModuleIO;
import frc.robot.modules.climb.ClimbModuleIOInputsAutoLogged;
import frc.robot.utils.LoggedTracer;

/**
 * 
 */
public class ClimbSubsystem extends SubsystemBase {
    private final String NAME = "Climb";

    /* Alerts */
    private final Alert disconnectedAlert = new Alert(NAME + " motor disconnected!", Alert.AlertType.kWarning);

    private final ClimbModuleIO climbModuleIO;
    protected final ClimbModuleIOInputsAutoLogged inputs;
    protected final Timer stateTimer;

    /**
     * 
     */
    public ClimbSubsystem(ClimbModuleIO climbModuleIO) {
        this.climbModuleIO = climbModuleIO;
        this.climbModuleIO.setBrakeMode(true);
        this.inputs = new ClimbModuleIOInputsAutoLogged();

        this.stateTimer = new Timer();
    }

    @Override
    public void periodic() {
        this.climbModuleIO.updateInputs(this.inputs);

        this.disconnectedAlert.set(!this.inputs.data.connected());

        Logger.processInputs(this.NAME, this.inputs);

        // Record cycle time
        LoggedTracer.record(this.NAME);
    }

    public Command deploy() {
        return run(() -> this.climbModuleIO.runTorqueCurrent(ClimbConstants.deployCurrent))
                .until(() -> this.inputs.data.positionRads() >= Units.degreesToRadians(ClimbConstants.deployAngle))
                .finallyDo(() -> this.climbModuleIO.runTorqueCurrent(0.0));
    }

    public Command climb() {
        Timer timer = new Timer();
        return runOnce(timer::restart)
                .andThen(
                        run(() -> {
                            boolean stopped = this.inputs.data.positionRads() >= Units
                                    .degreesToRadians(ClimbConstants.climbStopAngle);
                            if (stopped) {
                                timer.restart();
                            }
                            this.climbModuleIO.runTorqueCurrent(stopped
                                    ? 0.0
                                    : Math.min(ClimbConstants.climbCurrentRampRate * timer.get(),
                                            ClimbConstants.climbCurrent));
                        }))
                .finallyDo(() -> this.climbModuleIO.runTorqueCurrent(0.0));
    }
}
