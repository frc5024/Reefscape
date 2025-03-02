package frc.robot.modules.coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface CoralModuleIO {
    @AutoLog
    static class CoralIntakeIOInputs {
        public boolean connected = false;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(CoralIntakeIOInputs inputs) {
    }

    default void eject() {
    }

    default boolean hasCoral() {
        return false;
    }

    default void intake() {
    }

    default void setHasCoral(boolean has_coral) {
    }

    default void stop() {
    }
}
