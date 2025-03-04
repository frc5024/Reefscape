package frc.robot.modules.coral;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface CoralModuleIO {
    @AutoLog
    class CoralModuleIOInputs {
        public CoralModuleIOData data = new CoralModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record CoralModuleIOData(
            boolean connected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default void updateInputs(CoralModuleIOInputs inputs) {
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
