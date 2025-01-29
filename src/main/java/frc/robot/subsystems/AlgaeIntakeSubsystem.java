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
import frc.robot.modules.algae.AlgaeIntakeModuleIO;

/**
 * 
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final String NAME = "AlgaeIntake";

    public static enum Action {
        STOP, EJECT, INTAKE
    }

    private final AlgaeIntakeModuleIO intakeModule;
    private final Alert disconnected;

    protected final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();
    protected final Timer stateTimer = new Timer();

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    /**
     * 
     */
    public AlgaeIntakeSubsystem(AlgaeIntakeModuleIO intakeModule) {
        this.intakeModule = intakeModule;

        this.disconnected = new Alert(NAME + " motor disconnected!", Alert.AlertType.kWarning);

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>(NAME);
        this.stateMachine.setDefaultState(Action.STOP, this::handleStop);
        this.stateMachine.addState(Action.EJECT, this::handleEject);
        this.stateMachine.addState(Action.INTAKE, this::handleIntake);

        this.actionQueue = new LinkedList<Action>();

        this.stateTimer.start();
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
            this.intakeModule.eject();
        }
    }

    /**
     * 
     */
    protected void handleIntake(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.stateTimer.reset();
            this.stateTimer.start();
            this.intakeModule.intake();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.intakeModule.stop();
            this.stateTimer.stop();
        }
    }

    /**
     * 
     */
    public boolean hasAlgae() {
        return false;
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
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

        intakeModule.updateInputs(inputs);
        Logger.processInputs(NAME, inputs);

        this.disconnected.set(!inputs.connected);

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

        Logger.recordOutput("Subsystems/" + NAME + "/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/" + NAME + "/Has Algae", hasAlgae());
    }
}
