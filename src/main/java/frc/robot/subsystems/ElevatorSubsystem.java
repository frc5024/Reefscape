package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.modules.elevator.ElevatorModuleIO;

/**
 * 
 */
public class ElevatorSubsystem extends SubsystemBase {
    private final String NAME = "Elevator";
    private final Alert disconnected;

    public static enum Action {
        HOLD, MOVE_TO_IDLE, MOVE_TO_CORAL_1, MOVE_TO_CORAL_2, MOVE_TO_CORAL_3
    }

    private final ElevatorModuleIO elevatorModule;
    protected final ElevatorIOInputsAutoLogged inputs;
    protected final Timer stateTimer;

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    /**
     * 
     */
    public ElevatorSubsystem(ElevatorModuleIO elevatorModule) {
        this.elevatorModule = elevatorModule;
        this.inputs = new ElevatorIOInputsAutoLogged();
        this.disconnected = new Alert(NAME + " motor disconnected!", Alert.AlertType.kWarning);

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>(NAME);
        this.stateMachine.setDefaultState(Action.MOVE_TO_IDLE, this::handleMoveToIdle);
        this.stateMachine.addState(Action.HOLD, this::handleHold);
        this.stateMachine.addState(Action.MOVE_TO_CORAL_1, this::handleMoveToCoral1);

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
    protected void handleHold(StateMetadata<Action> stateMetadata) {
    }

    /**
     * 
     */
    protected void handleMoveToIdle(StateMetadata<Action> stateMetadata) {
    }

    /**
     * 
     */
    protected void handleMoveToCoral1(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorModule.runOpenLoop(Units.inchesToMeters(5));
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
            default:
                return this.elevatorModule.isAtDistance() || !this.stateTimer.isRunning();
        }
    }

    /**
     * 
     */
    public void periodic() {
        this.stateMachine.update();

        this.elevatorModule.updateInputs(this.inputs);
        Logger.processInputs(this.NAME, this.inputs);

        this.disconnected.set(!this.inputs.connected);

        // actions run for no longer than 3 seconds
        if (this.stateTimer.isRunning() && this.stateTimer.hasElapsed(3)) {
            this.stateTimer.stop();
        }

        if (isActionComplete()) {
            this.stateMachine.setState(Action.HOLD);
        }

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.HOLD && this.actionQueue.size() > 0) {
            try {
                Action nextAction = this.actionQueue.removeFirst();
                this.stateMachine.setState(nextAction);
            } catch (NoSuchElementException e) {
            }
        }

        Logger.recordOutput("Subsystems/" + this.NAME + "/Current State", this.stateMachine.getCurrentState());
    }
}
