package frc.robot.controls;

import java.util.function.Supplier;

/**
 * 
 */
public class GameData {
    /* Drive to which coral pole */
    public enum CoralPole {
        LEFT, RIGHT
    }

    /* Drive for which game piece */
    public enum DriveMode {
        ALGAE, CORAL
    }

    public static GameData instance;

    private int reefStationIndex = 1;
    private CoralPole coralPole = CoralPole.LEFT;
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
    public Supplier<CoralPole> getCoralPole() {
        return () -> this.coralPole;
    }

    public Supplier<DriveMode> getDriveMode() {
        return () -> this.driveMode;
    }

    public String getCoralPoleAsString() {
        switch (this.coralPole) {
            case LEFT:
                return "LEFT";

            case RIGHT:
                return "RIGHT";

            default:
                return "";
        }
    }

    public String getDriveModeAsString() {
        switch (this.driveMode) {
            case ALGAE:
                return "ALGAE";

            case CORAL:
                return "CORAL";

            default:
                return "";
        }
    }

    public int getReefStationIndex() {
        return reefStationIndex;
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

    public void setCoralPole(CoralPole coralPole) {
        this.coralPole = coralPole;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void setReefStationIndex(int reefStationIndex) {
        this.reefStationIndex += reefStationIndex;

        if (this.reefStationIndex > 6)
            this.reefStationIndex = 6;
        if (this.reefStationIndex < 1)
            this.reefStationIndex = 1;
    }
}
