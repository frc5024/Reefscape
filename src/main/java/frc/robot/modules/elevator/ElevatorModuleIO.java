package frc.robot.modules.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ElevatorModuleIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] currentAmps = new double[] {};
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

    default void runOpenLoop(double output) {
    }

    default void runVolts(double volts) {
    }

    default void stop() {
    }

    /**
     * Run elevator output shaft to positionRad with additional feedforward output
     */
    default void runPosition(double positionRad, double feedforward) {
    }

    default void setBrakeMode(boolean enabled) {
    }

    default void updatePID(double kP, double kI, double kD) {
    }
}
