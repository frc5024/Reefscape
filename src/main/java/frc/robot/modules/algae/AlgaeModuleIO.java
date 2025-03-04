package frc.robot.modules.algae;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface AlgaeModuleIO {
    @AutoLog
    class AlgaeModuleIOInputs {
        public AlgaeModuleIOData data = new AlgaeModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record AlgaeModuleIOData(
            boolean connected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default void updateInputs(AlgaeModuleIOInputs inputs) {
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
