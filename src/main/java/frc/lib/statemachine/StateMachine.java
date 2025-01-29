package frc.lib.statemachine;

import java.util.HashMap;
import java.util.function.Consumer;

public class StateMachine<T> {
    // The state to be run in the event of an error with state handling. Should be an idle state
    public T defaultStateKey;

    // A mapping of all states to their key
    public HashMap<T, StateHandler<T>> states = new HashMap<>();

    // Tracker for the last state to be run
    public T lastStateKey;
    public T desiredStateKey;

    // Telemetry data table
    private String name;

    /**
     * 
     */
    public StateMachine(String name) {
        this.name = name;
    }

    /**
     * Add a state to the StateMachine
     * 
     * @param key    State key
     * @param action Action to be run
     */
    public void addState(T key, Consumer<StateMetadata<T>> action) {
        StateHandler<T> handler = new StateHandler<T>(key, this, action);

        this.states.put(key, handler);
    }

    /**
     * Add a state to the StateMachine, and set it as the default. This state will
     * be called upon any error in state handling, or when no state is set.
     * 
     * @param key    State key
     * @param action Action to be run
     */
    public void setDefaultState(T key, Consumer<StateMetadata<T>> action) {
        addState(key, action);

        defaultStateKey = key;

        setState(key);
    }

    /**
     * Remove a state from the StateMachine
     * 
     * @param key State key
     */
    public void removeState(T key) {
        this.states.remove(key);

        if (defaultStateKey == key) {
            defaultStateKey = null;
        }
    }

    /**
     * Update the machine. This MUST be called periodically
     */
    public void update() {
        if (desiredStateKey == null && defaultStateKey == null) {
            return;
        }

        boolean defaultStateWasOverridden = false;
        if (desiredStateKey == null) {
            desiredStateKey = defaultStateKey;
            defaultStateWasOverridden = true;
        }

        // If the current, and last state keys differ, this is the first run of the state
        boolean isNew = lastStateKey == null || desiredStateKey == null || defaultStateWasOverridden || !lastStateKey.equals(desiredStateKey);

        lastStateKey = desiredStateKey;

        StateHandler<T> state = states.get(desiredStateKey);
        state.call(isNew, lastStateKey);
    }

    /**
     * Set the StateMachine's state
     */
    public void setState(T key) {
        desiredStateKey = key;
    }

    /**
     * Get the system's current state
     * 
     * @return current state
     */
    public T getCurrentState() {
        return desiredStateKey;
    }
}
