package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.algae.AlgaeIntakeIOInputsAutoLogged;
import frc.robot.modules.algae.AlgaeModuleIO;

/**
 * 
 */
public class AlgaeSubsystem extends SubsystemBase {
    private final String NAME = "Algae";
    private final Alert disconnected;

    public static enum Action {
        STOP, EJECT, INTAKE
    }

    private final AlgaeModuleIO algaeModuleIO;
    protected final AlgaeIntakeIOInputsAutoLogged inputs;
    protected final Timer stateTimer;

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    /**
     * 
     */
    public AlgaeSubsystem(AlgaeModuleIO algaeModuleIO) {
        this.algaeModuleIO = algaeModuleIO;
        this.inputs = new AlgaeIntakeIOInputsAutoLogged();
        this.disconnected = new Alert(NAME + " motor disconnected!", Alert.AlertType.kWarning);

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>(NAME);
        this.stateMachine.setDefaultState(Action.STOP, this::handleStop);
        this.stateMachine.addState(Action.EJECT, this::handleEject);
        this.stateMachine.addState(Action.INTAKE, this::handleIntake);

        this.actionQueue = new LinkedList<Action>();

        this.stateTimer = new Timer();
    }

    /**
     * 
     */
    public void addAction(Action action) {
        this.actionQueue.add(action);
    }

    /**
     * 
     */
    public Action getCurrentState() {
        return this.stateMachine.getCurrentState();
    }

    /**
     * 
     */
    protected void handleEject(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.stateTimer.reset();
            this.stateTimer.start();
            this.algaeModuleIO.eject();
        }
    }

    /**
     * 
     */
    protected void handleIntake(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.stateTimer.reset();
            this.stateTimer.start();
            this.algaeModuleIO.intake();

            LEDSubsystem.getInstance().strobeAqua();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.algaeModuleIO.stop();
            this.stateTimer.stop();

            if (hasAlgae()) {
                LEDSubsystem.getInstance().solidAqua();
            } else {
                LEDSubsystem.getInstance().solidBlack();
            }
        }
    }

    /**
     * 
     */
    public boolean hasAlgae() {
        return this.algaeModuleIO.hasAlgae();
    }

    /**
     * 
     */
    public boolean hasEjected() {
        return !hasAlgae();
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
            case EJECT:
                return !this.stateTimer.isRunning() || hasEjected();
            case INTAKE:
                return !this.stateTimer.isRunning() || hasAlgae();
            default:
                return !this.stateTimer.isRunning();
        }
    }

    /**
     * 
     */
    public void periodic() {
        this.stateMachine.update();

        this.algaeModuleIO.updateInputs(this.inputs);
        Logger.processInputs(this.NAME, this.inputs);

        this.disconnected.set(!this.inputs.connected);

        // actions run for no longer than 3 seconds
        if (this.stateTimer.isRunning() && this.stateTimer.hasElapsed(3)) {
            this.stateTimer.stop();
        }

        if (isActionComplete()) {
            this.stateMachine.setState(Action.STOP);
        }

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.STOP && this.actionQueue.size() > 0) {
            try {
                Action nextAction = this.actionQueue.removeFirst();
                this.stateMachine.setState(nextAction);
            } catch (NoSuchElementException e) {
            }
        }

        Logger.recordOutput("Subsystems/" + this.NAME + "/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/" + this.NAME + "/Has Algae", hasAlgae());
    }

    /**
     * Used in autonomous simulations
     */
    public void setHasAlgae(boolean has_algae) {
        this.algaeModuleIO.setHasAlgae(has_algae);
    }
}
