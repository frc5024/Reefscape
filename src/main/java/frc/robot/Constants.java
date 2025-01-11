package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * 
 */
public final class Constants {
    // Set to true to use FeedForwardCharacterization and
    // WheelRadiusCharacterization auto commands
    public static final boolean TUNING_MODE = true;

    // AdvantageKit simulation
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL, // Running on a real robot
        SIM, // Running a physics simulator
        REPLAY // Replaying from a log file
    }
}