package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

/** Utility class for logging code execution times. */
public class LoggedTracer {
    private LoggedTracer() {
    }

    private static double startTime = -1.0;

    /** Reset the clock. */
    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    /** Save the time elapsed since the last reset or record. */
    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
        startTime = now;
    }
}
