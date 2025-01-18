package frc.robot;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.List;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.camera.Camera;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
/**
 * 
 */
public final class Constants {
    /**
     * 
     */
    public static final class RobotConstants {
        public static final double LENGTH_INCHES = 38;
        public static final double LENGTH_METERS = Units.inchesToMeters(LENGTH_INCHES);

        // Set to true to use FeedForwardCharacterization and
        // WheelRadiusCharacterization auto commands
        public static final boolean TUNING_MODE = true;
    }

    // AdvantageKit simulation
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL, // Running on a real robot
        SIM, // Running a physics simulator
        REPLAY // Replaying from a log file
    }

    // PathPlanner Config Constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    public static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1)
                            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            SwerveDriveSubsystem.getModuleTranslations());

    /**
     * Maple Sim Constants
     */
    public static final int driveMotorCurrentLimit = 60;
    public static final int turnMotorCurrentLimit = 20;

    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(SwerveDriveSubsystem.getModuleTranslations())
            .withRobotMass(Kilogram.of(ROBOT_MASS_KG))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getNeoVortex(1),
                    DCMotor.getNeo550(1),
                    (45.0 * 22.0) / (14.0 * 15.0),
                    9424.0 / 203.0,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(0.02),
                    WHEEL_COF));

    /**
     * 
     */
    public static class FieldConstants {
        public static final double LENGTH_METERS = Units.inchesToMeters(689.0);
        public static final double WIDTH_METERS = Units.inchesToMeters(323.0);

        // starting poses for game mode for blue alliance station 1, 2, 3
        public static final Pose2d[] STATION_POSES = new Pose2d[] {
                new Pose2d(8.217, 7.272, Rotation2d.fromDegrees(180.0)),
                new Pose2d(8.217, 6.166, Rotation2d.fromDegrees(180.0)),
                new Pose2d(8.217, 5.074, Rotation2d.fromDegrees(180.0))
        };

        // starting poses for tuning mode for station 1, 2, 3
        public static final Pose2d[] TUNING_POSES = new Pose2d[] {
                new Pose2d(8.217, 7.272, Rotation2d.fromDegrees(180.0)),
                new Pose2d(8.217, 6.166, Rotation2d.fromDegrees(180.0)),
                new Pose2d(8.217, 5.074, Rotation2d.fromDegrees(180.0))
        };
    }

    /**
     * 
     */
    public static class VisionConstants {
        public static AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        /**
         * TODO: set camera names and positions - pitch is based on degress from
         * vertical
         */
        public static final Camera FRONT_CAMERA = new Camera("Arducam_OV9281-2",
                Camera.Type.APRILTAG, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(6.0),
                0.0, Units.degreesToRadians(0), 0.0);
        public static final Camera REAR_CAMERA = new Camera("Arducam_OV9281-1",
                Camera.Type.APRILTAG, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(0.0), Units.inchesToMeters(0), Units.inchesToMeters(6.0),
                0.0, Units.degreesToRadians(0), Math.PI);
        public static final Camera GAME_PIECE_CAMERA = new Camera("WebCam",
                Camera.Type.COLOURED_SHAPE, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(0.0), 0.0, Units.inchesToMeters(0.0),
                0.0, Units.degreesToRadians(0), 0.0);

        /**
         * TODO: set list of enabled camera
         */
        public static final List<Camera> CAMERAS = Arrays.asList(FRONT_CAMERA, REAR_CAMERA);
        public static final String DATA_FROM_CAMERA = REAR_CAMERA.getName();

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.3;
        public static final double APRILTAG_MAX_Z_ERROR = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
        public static double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
        public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] CAMERA_STD_DEV_FACTORS = new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
        };

        // for simulation only
        public static final double DIAGONAL_FOV = 70;
        public static final int IMG_WIDTH = 960;
        public static final int IMG_HEIGHT = 720;
    }
}