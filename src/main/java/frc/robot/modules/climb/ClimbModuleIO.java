package frc.robot.modules.climb;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ClimbModuleIO {
    @AutoLog
    class ClimbModuleIOInputs {
        public ClimbModuleIOData data = new ClimbModuleIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ClimbModuleIOData(
            boolean connected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius) {
    }

    default void updateInputs(ClimbModuleIOInputs inputs) {
    }

    default void runTorqueCurrent(double current) {
    }

    default void setBrakeMode(boolean enabled) {
    }
}
