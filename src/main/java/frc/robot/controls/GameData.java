package frc.robot.controls;

import java.util.function.Supplier;

/**
 * 
 */
public class GameData {
    public enum GamePieceMode {
        ALGAE, CORAL
    }

    public enum CoralPole {
        LEFT, RIGHT
    }

    public enum CoralLevel {
        L1, L2, L3, L4;

        static public final CoralLevel[] values = values();

        public CoralLevel previous() {
            return values[(ordinal() - 1 + values.length) % values.length];
        }

        public CoralLevel next() {
            return values[(ordinal() + 1) % values.length];
        }
    }

    public enum ReefStation {
        S1, S2, S3, S4, S5, S6;

        static public final ReefStation[] values = values();

        public ReefStation previous() {
            return values[(ordinal() - 1 + values.length) % values.length];
        }

        public ReefStation next() {
            return values[(ordinal() + 1) % values.length];
        }
    }

    public static GameData instance;

    private CoralPole coralPole = CoralPole.LEFT;
    private CoralLevel coralLevel = CoralLevel.L4;
    private ReefStation reefStation = ReefStation.S1;
    private GamePieceMode gamePieceMode = GamePieceMode.CORAL;

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
    public void toggleGamePieceMode() {
        if (this.gamePieceMode == GamePieceMode.ALGAE) {
            this.gamePieceMode = GamePieceMode.CORAL;
        } else {
            this.gamePieceMode = GamePieceMode.ALGAE;
        }
    }

    /**
     * Getters and Setters
     */
    public CoralLevel getCoralLevel() {
        return this.coralLevel;
    }

    public Supplier<CoralPole> getCoralPole() {
        return () -> this.coralPole;
    }

    public Supplier<GamePieceMode> getGamePieceMode() {
        return () -> this.gamePieceMode;
    }

    public String getCoralLevelAsString() {
        switch (this.coralLevel) {
            case L1:
                return "L1";

            case L2:
                return "L2";

            case L3:
                return "L3";

            case L4:
                return "L4";

            default:
                return "";
        }
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

    public String getGamePieceModeAsString() {
        switch (this.gamePieceMode) {
            case ALGAE:
                return "ALGAE";

            case CORAL:
                return "CORAL";

            default:
                return "";
        }
    }

    public ReefStation getReefStation() {
        return this.reefStation;
    }

    public String getReefStationAsString() {
        switch (this.reefStation) {
            case S1:
                return "S1";
            case S2:
                return "S2";
            case S3:
                return "S3";
            case S4:
                return "S4";
            case S5:
                return "S5";
            case S6:
                return "S6";
            default:
                return "";
        }
    }

    public int getReefStationAsInt() {
        switch (this.reefStation) {
            case S1:
                return 1;
            case S2:
                return 2;
            case S3:
                return 3;
            case S4:
                return 4;
            case S5:
                return 5;
            case S6:
                return 6;
            default:
                return -1;
        }
    }

    public void nextCoralLevel() {
        this.coralLevel = this.coralLevel.next();
    }

    public void previousCoralLevel() {
        this.coralLevel = this.coralLevel.previous();
    }

    public void setCoralPole(CoralPole coralPole) {
        this.coralPole = coralPole;
    }

    public void setGamePieceMode(GamePieceMode gamePieceMode) {
        this.gamePieceMode = gamePieceMode;
    }

    public void nextReefStation() {
        this.reefStation = this.reefStation.next();
    }

    public void previousReefStation() {
        this.reefStation = this.reefStation.previous();
    }
}
