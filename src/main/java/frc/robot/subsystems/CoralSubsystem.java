package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.coral.CoralModuleIO;
import frc.robot.modules.coral.CoralModuleIOInputsAutoLogged;
import frc.robot.utils.LoggedTracer;

/**
 * 
 */
public class CoralSubsystem extends SubsystemBase {
    private final String NAME = "Coral";

    /* Alerts */
    private final Alert topDisconnectedAlert = new Alert(NAME + " top motor disconnected!", Alert.AlertType.kWarning);
    private final Alert bottomDisconnectedAlert = new Alert(NAME + " bottom motor disconnected!",
            Alert.AlertType.kWarning);

    public static enum Action {
        STOP, EJECT, INTAKE
    }

    private final CoralModuleIO coralModuleIO;
    protected final CoralModuleIOInputsAutoLogged inputs;
    protected final Timer stateTimer;

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    /**
     * 
     */
    public CoralSubsystem(CoralModuleIO coralModuleIO) {
        this.coralModuleIO = coralModuleIO;
        this.inputs = new CoralModuleIOInputsAutoLogged();

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
            this.coralModuleIO.eject();
        }
    }

    /**
     * 
     */
    protected void handleIntake(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.stateTimer.reset();
            this.stateTimer.start();
            this.coralModuleIO.intake();

            LEDSubsystem.getInstance().strobeWhite();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.coralModuleIO.stop();
            this.stateTimer.stop();

            if (hasCoral()) {
                LEDSubsystem.getInstance().solidWhite();
            } else {
                LEDSubsystem.getInstance().solidBlack();
            }
        }
    }

    /**
     * 
     */
    public boolean hasCoral() {
        return this.coralModuleIO.hasCoral();
    }

    /**
     * 
     */
    public boolean hasEjected() {
        return !hasCoral();
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
            case EJECT:
                return !this.stateTimer.isRunning() || hasEjected();
            case INTAKE:
                return !this.stateTimer.isRunning() || hasCoral();
            default:
                return !this.stateTimer.isRunning();
        }
    }

    /**
     * 
     */
    public void periodic() {
        this.stateMachine.update();

        this.coralModuleIO.updateInputs(this.inputs);
        Logger.processInputs(this.NAME, this.inputs);

        this.topDisconnectedAlert.set(!this.inputs.data.topMotorconnected());
        this.topDisconnectedAlert.set(!this.inputs.data.bottomMotorconnected());

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
        Logger.recordOutput("Subsystems/" + this.NAME + "/Has Coral", this.hasCoral());

        // Record cycle time
        LoggedTracer.record(this.NAME);
    }

    /**
     * Used in autonomous simulations
     */
    public void setHasCoral(boolean has_coral) {
        this.coralModuleIO.setHasCoral(has_coral);
    }
}