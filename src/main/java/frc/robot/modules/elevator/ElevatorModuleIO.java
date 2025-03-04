package frc.robot.modules.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ElevatorModuleIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean connected = false;
        public double positionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double[] appliedVoltage = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    default void updateInputs(ElevatorIOInputs inputs) {
    }

    default boolean isAtBottom() {
        return true;
    }

    default boolean isAtTop() {
        return true;
    }

    default public void runCharacterization(double output) {
    }

    default void runOpenLoop(double output) {
    }

    default void runVolts(double volts) {
    }

    default void stop() {
    }

    /**
     * Run elevator output shaft to positionRad with additional feedforward output
     */
    default void runPosition(double positionRad) {
    }

    default void setBrakeMode(boolean enabled) {
    }

    default void updatePID(double kP, double kI, double kD) {
    }
}
