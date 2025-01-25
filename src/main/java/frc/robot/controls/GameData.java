package frc.robot.controls;

import java.util.function.Supplier;

/**
 * 
 */
public class GameData {
    public static GameData instance;

    /* Index to hold which station/pole to drive to */
    private int reefStationIndex;
    private int reefPoleIndex;

    /* Drive for which game piece */
    public enum DriveMode {
        ALGAE, CORAL
    }

    private DriveMode driveMode = DriveMode.CORAL;

    /**
     * 
     */
    public static GameData getInstance() {
        if (instance == null) {
            instance = new GameData();
        }

        return instance;
    }

    /**
     * Gets the current game piece mode.
     *
     * @return The supplier that provides the current game piece mode.
     */
    public Supplier<DriveMode> getDriveMode() {
        return () -> this.driveMode;
    }

    /**
     * Toggles the game piece mode between ALGAE and CORAL.
     */
    public void toggleDriveMode() {
        if (this.driveMode == DriveMode.ALGAE) {
            this.driveMode = DriveMode.CORAL;
        } else {
            this.driveMode = DriveMode.ALGAE;
        }
    }

    /**
     * Getters and Setters
     */
    public String getDriveModeAsString() {
        switch (getDriveMode().get()) {
            case ALGAE:
                return "ALGAE";

            case CORAL:
                return "CORAL";

            default:
                return "";
        }
    }

    public int getReefPoleIndex() {
        return reefPoleIndex;
    }

    public int getReefStationIndex() {
        return reefStationIndex;
    }

    public String getReefPoleIndexAsString() {
        return this.reefPoleIndex == 1 ? "LEFT" : "RIGHT";
    }

    public String getReefStationIndexAsString() {
        switch (this.reefStationIndex) {
            case 1:
                return "ONE";
            case 2:
                return "TWO";
            case 3:
                return "THREE";
            case 4:
                return "FOUR";
            case 5:
                return "FIVE";
            case 6:
                return "SIX";
            default:
                return "";
        }
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void setReefIndexes(int reefStationIndex, int reefPoleIndex) {
        this.reefStationIndex += reefStationIndex;
        this.reefPoleIndex += reefPoleIndex;

        if (this.reefStationIndex > 6)
            this.reefStationIndex = 6;
        if (this.reefStationIndex < 1)
            this.reefStationIndex = 1;

        if (this.reefPoleIndex > 2)
            this.reefPoleIndex = 2;
        if (this.reefPoleIndex < 1)
            this.reefPoleIndex = 1;
    }
}
