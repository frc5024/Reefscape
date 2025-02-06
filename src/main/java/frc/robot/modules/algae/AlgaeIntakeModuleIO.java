package frc.robot.modules.algae;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface AlgaeIntakeModuleIO {
    @AutoLog
    static class AlgaeIntakeIOInputs {
        public boolean connected = false;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(AlgaeIntakeIOInputs inputs) {
    }

    default void eject() {
    }

    default boolean hasAlgae() {
        return false;
    }

    default void intake() {
    }

    default void setHasAlgae(boolean has_algae) {
    }

    default void stop() {
    }
}
