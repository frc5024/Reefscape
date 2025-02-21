package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.RobotConstants;
import frc.robot.modules.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.modules.elevator.ElevatorModuleIO;
import frc.robot.modules.elevator.ElevatorVisualizer;
import frc.robot.utils.EqualsUtil;

/**
 * 
 */
public class ElevatorSubsystem extends SubsystemBase {
    private final String NAME = "Elevator";
    private final Alert disconnected;

    public static enum Action {
        STOP, MOVE_TO_BOTTOM, MOVE_TO_ALGAE_1, MOVE_TO_ALGAE_2, MOVE_TO_PROCESSOR, MOVE_TO_CORAL_1, MOVE_TO_CORAL_2,
        MOVE_TO_CORAL_3, MOVE_TO_CORAL_4
    }

    private final ElevatorModuleIO elevatorModule;
    protected final ElevatorIOInputsAutoLogged inputs;
    protected final Timer stateTimer;

    /* Mechanisim2d Display for Monitoring the Elevator Position */
    private final ElevatorVisualizer elevatorVisualizer = ElevatorVisualizer.getInstance("Measured");

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;
    private ExponentialProfile profile;
    private Supplier<State> goal;
    private boolean atGoal;
    private State setpoint;
    private ElevatorLevel elevatorLevel;

    private final Supplier<Boolean> hasAlgaSupplier;
    private final Supplier<Boolean> hasCoralSupplier;

    /**
     * 
     */
    public ElevatorSubsystem(ElevatorModuleIO elevatorModule, Supplier<Boolean> hasAlgaSupplier,
            Supplier<Boolean> hasCoralSupplier) {
        this.elevatorModule = elevatorModule;
        this.hasAlgaSupplier = hasAlgaSupplier;
        this.hasCoralSupplier = hasCoralSupplier;

        this.inputs = new ElevatorIOInputsAutoLogged();
        this.disconnected = new Alert(NAME + " motor disconnected!", Alert.AlertType.kWarning);

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>(NAME);
        this.stateMachine.setDefaultState(Action.MOVE_TO_BOTTOM, this::handleMoveToBottom);
        this.stateMachine.addState(Action.STOP, this::handleStop);
        this.stateMachine.addState(Action.MOVE_TO_ALGAE_1, this::handleMoveToAlgae1);
        this.stateMachine.addState(Action.MOVE_TO_ALGAE_2, this::handleMoveToAlgae2);
        this.stateMachine.addState(Action.MOVE_TO_PROCESSOR, this::handleMoveToProcessor);
        this.stateMachine.addState(Action.MOVE_TO_CORAL_1, this::handleMoveToCoral1);
        this.stateMachine.addState(Action.MOVE_TO_CORAL_2, this::handleMoveToCoral2);
        this.stateMachine.addState(Action.MOVE_TO_CORAL_3, this::handleMoveToCoral3);
        this.stateMachine.addState(Action.MOVE_TO_CORAL_4, this::handleMoveToCoral4);

        this.actionQueue = new LinkedList<Action>();

        this.stateTimer = new Timer();

        this.profile = new ExponentialProfile(fromMaxTorque(ElevatorConstants.MAX_TORQUE));
        this.goal = State::new;
        this.atGoal = false;
        this.setpoint = new State();
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
    protected void handleMoveToAlgae1(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.AlgaeL1;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToAlgae2(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.AlgaeL2;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToBottom(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.Bottom;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToProcessor(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.Processor;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToCoral1(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.CoralL1;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToCoral2(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.CoralL2;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToCoral3(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.CoralL3;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleMoveToCoral4(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorLevel = ElevatorLevel.CoralL4;
            setGoal(this.elevatorLevel.heightInMeters);
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.elevatorModule.stop();
            this.stateTimer.start();
        }
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        switch (this.stateMachine.getCurrentState()) {
            default:
                return this.atGoal || !this.stateTimer.isRunning();
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

        State goalState = new State(MathUtil.clamp(this.goal.get().position, 0.0, ElevatorConstants.HEIGHT_IN_METERS),
                this.goal.get().velocity);
        this.setpoint = profile.calculate(RobotConstants.LOOP_PERIOD_SECS, setpoint, goalState);

        this.elevatorModule.runPosition(
                setpoint.position / ElevatorConstants.drumRadiusMeters,
                ElevatorConstants.kS * Math.signum(setpoint.velocity) // Magnitude irrelevant
                        + ElevatorConstants.kG * ElevatorConstants.ANGLE.getSin());

        // Check at goal
        this.atGoal = EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
                && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

        // actions run for no longer than 5 seconds
        if (this.stateTimer.isRunning() && this.stateTimer.hasElapsed(5)) {
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

        /**
         * Stop motors if elevator reaches top or bottom
         */
        if (this.elevatorModule.isAtBottom() || this.elevatorModule.isAtTop()) {

        }

        this.elevatorVisualizer.update(getPositionMeters(), this.elevatorLevel, this.hasAlgaSupplier.get(),
                this.hasCoralSupplier.get());

        Logger.recordOutput("Subsystems/" + this.NAME + "/AtGoal", this.atGoal);
        Logger.recordOutput("Subsystems/" + this.NAME + "/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/" + this.NAME + "/MeasuredHeight", getPositionMeters());
        Logger.recordOutput("Subsystems/" + this.NAME + "/SetpointPositionMeters", setpoint.position);
        Logger.recordOutput("Subsystems/" + this.NAME + "/SetpointVelocityMetersPerSec", setpoint.velocity);
        Logger.recordOutput("Subsystems/" + this.NAME + "/GoalPositionMeters", goalState.position);
        Logger.recordOutput("Subsystems/" + this.NAME + "/GoalVelocityMetersPerSec", goalState.velocity);
    }

    public double getPositionMeters() {
        return (inputs.positionRad - 0.0) * ElevatorConstants.drumRadiusMeters;
    }

    public void setGoal(double goal) {
        setGoal(() -> new State(goal, 0.0));
    }

    public void setGoal(Supplier<State> goal) {
        this.goal = goal;
        this.atGoal = false;
    }

    private static Constraints fromMaxTorque(double maxTorque) {
        return Constraints.fromStateSpace(
                maxTorque / ElevatorConstants.gearbox.KtNMPerAmp,
                ElevatorConstants.A.get(1, 1),
                ElevatorConstants.B.get(1));
    }

    /**
     * Used in tuning mode
     */
    public void updatePID(double kP, double kI, double kD) {
        this.elevatorModule.updatePID(kP, kI, kD);
    }
}
