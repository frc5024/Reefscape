package frc.robot.modules.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface ElevatorModuleIO {
    @AutoLog
    class ElevatorIOInputs {
        public ElevatorModuleIOData data = new ElevatorModuleIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);
    }

    record ElevatorModuleIOData(
            boolean connected,
            boolean followerConnected,
            double positionRads,
            double velocityRadsPerSec,
            double appliedVoltage,
            double torqueCurrentAmps,
            double supplyCurrentAmps,
            double tempCelsius,
            double followerAppliedVolts,
            double followerTorqueCurrentAmps,
            double followerSupplyCurrentAmps,
            double followerTempCelsius) {
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
