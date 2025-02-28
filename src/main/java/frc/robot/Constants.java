package frc.robot;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.List;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.camera.Camera;
import frc.robot.utils.COTSTalonFXSwerveConstants;
import frc.robot.utils.COTSTalonFXSwerveConstants.SDS.MK4i;

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
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MASS_KG = 74.088;
        public static final double WHEEL_COF = 1.2;

        // Set to true to use FeedForwardCharacterization and
        // WheelRadiusCharacterization auto commands
        public static final boolean TUNING_MODE = false;

        // AdvantageKit simulation
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

        public static enum Mode {
            REAL, // Running on a real robot
            SIM, // Running a physics simulator
            REPLAY // Replaying from a log file
        }
    }

    /**
     * 
     */
    public static class ElevatorConstants {
        public static final double HEIGHT_IN_METERS = 2.0;
        public static final double MAX_TORQUE = 20.0;
        public static final double MAX_UP_SPEED = 0.2;
        public static final double MAX_DOWN_SPEED = 0.1;
        public static final Rotation2d ANGLE = Rotation2d.fromDegrees(90.0);
        public static final double kS = 5.0;
        public static final double kG = 50.0;

        // Distance elevator must travel to align with outtake
        public enum ElevatorLevel {
            Bottom(0.0, 0.0),
            AlgaeL1(Units.inchesToMeters(10.5), 0.0),
            AlgaeL2(Units.inchesToMeters(18.0), 0.0),
            Processor(Units.inchesToMeters(0.0), 0.0),

            CoralL1(Units.inchesToMeters(0), 0.0),
            CoralL2(Units.inchesToMeters(10.0), -35.0),
            CoralL3(Units.inchesToMeters(18.0), -35.0),
            CoralL4(Units.inchesToMeters(30.0), -90.0);

            ElevatorLevel(double heightInMeters, double angleInDegrees) {
                this.heightInMeters = heightInMeters;
                this.angleInDegrees = angleInDegrees;
            }

            public final double heightInMeters;
            public final double angleInDegrees;
        }

        public static final double drumRadiusMeters = Units.inchesToMeters(6.0);
        public static final double reduction = 5.0;
        public static final double carriageMassKg = Units.lbsToKilograms(6.0);
        public static final double stagesMassKg = Units.lbsToKilograms(12.0);
        public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(2).withReduction(reduction);

        public static final Matrix<N2, N2> A = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 1, 0,
                -gearbox.KtNMPerAmp
                        / (gearbox.rOhms
                                * Math.pow(drumRadiusMeters, 2)
                                * (carriageMassKg + stagesMassKg)
                                * gearbox.KvRadPerSecPerVolt));

        public static final Vector<N2> B = VecBuilder.fill(
                0.0, gearbox.KtNMPerAmp / (drumRadiusMeters * (carriageMassKg + stagesMassKg)));
    }

    /**
     * 
     */
    public static class FieldConstants {
        public static final double LENGTH_METERS = Units.inchesToMeters(693.0);
        public static final double WIDTH_METERS = Units.inchesToMeters(318.0);
        public static final double REEF_POLE_OFFSET = Units.inchesToMeters(12.94 / 2);

        // starting poses for game mode for blue/red alliance station 1, 2, 3
        public static final Pose2d[][] STATION_POSES = new Pose2d[][] {
                {
                        // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(8.217, 7.272, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(8.217, 6.166, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(8.217, 5.074, Rotation2d.fromDegrees(180.0))
                },
                {
                        // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(9.383, 0.805, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(9.383, 1.991, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(9.383, 3.003, Rotation2d.fromDegrees(0.0))
                }
        };

        // starting poses for tuning mode for station 1, 2, 3
        public static final Pose2d[][] TUNING_POSES = new Pose2d[][] {
                {
                        new Pose2d(8.217, 7.272, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(8.217, 6.166, Rotation2d.fromDegrees(180.0)),
                        new Pose2d(8.217, 5.074, Rotation2d.fromDegrees(180.0))
                },
                {
                        new Pose2d(9.383, 0.805, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(9.383, 1.991, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(9.383, 3.003, Rotation2d.fromDegrees(0.0))
                }
        };

        // left and right coral station poses as seen from driver's station
        public static final Pose2d[] CORAL_STATION_POSES = new Pose2d[] {
                new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Rotation2d.fromDegrees(306.0)),
                new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Rotation2d.fromDegrees(54.0))
        };

        // left and right coral station poses as seen from driver's station for dropping
        // coral in simulation
        public static final Pose2d[] CORAL_DROP_STATIONS = new Pose2d[] {
                new Pose2d(Units.inchesToMeters(48.0), Units.inchesToMeters(6.0),
                        new Rotation2d(Units.degreesToRadians(55.0))),
                new Pose2d(Units.inchesToMeters(48.0), Units.inchesToMeters(312.0),
                        new Rotation2d(Units.degreesToRadians(-55.0)))
        };

        // starts with one closest to driver station and rotates clockwise
        public static final Pose2d[] REEF_POSES = new Pose2d[] {
                new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0.0)), // Tag-18
                new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),
                        Rotation2d.fromDegrees(-60.0)), // Tag-19
                new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),
                        Rotation2d.fromDegrees(-120.0)), // Tag-20
                new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50),
                        Rotation2d.fromDegrees(180.0)), // Tag-21
                new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),
                        Rotation2d.fromDegrees(120.0)), // Tag-22
                new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(60.0)) // Tag-17
        };
    }

    /**
     * Gyro Constants
     */
    public static class GyroContants {
        public static final double SCALE_VALUE = 3600.0 / 3831.020004272461;
    }

    /**
     * Maple Sim Constants
     */
    public static class MapleSimConstants {
        public static final int driveMotorCurrentLimit = 60;
        public static final int turnMotorCurrentLimit = 20;

        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.00865;
        private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
        public static final double driveSimKv = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT); // 0.0789;

        public static final double turnSimP = 8.0;
        public static final double turnSimD = 0.0;

        public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
                .withCustomModuleTranslations(SwerveConstants.moduleTranslations)
                .withRobotMass(Kilogram.of(RobotConstants.MASS_KG))
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getNeoVortex(1),
                        DCMotor.getNeo550(1),
                        (45.0 * 22.0) / (14.0 * 15.0),
                        9424.0 / 203.0,
                        Volts.of(0.1),
                        Volts.of(0.1),
                        Meters.of(SwerveConstants.cotsDriveConstants.wheelDiameter / 2),
                        KilogramSquareMeters.of(0.02),
                        RobotConstants.WHEEL_COF));
    }

    /**
     * 
     */
    public static final class MechanismConstants {
        public static final double CANVAS_SIZE_METERS = Units.inchesToMeters(98);
        public static final LoggedMechanism2d CANVAS = new LoggedMechanism2d(CANVAS_SIZE_METERS, CANVAS_SIZE_METERS,
                new Color8Bit(Color.kWheat));
    }

    /**
     *
     */
    public static final class PIDConstants {
        // PID constants for swerve modules for drive motor. Turn pid is set in cots
        // constants
        public static final double SWERVE_MODULE_DRIVE_KP = 0.1;
        public static final double SWERVE_MODULE_DRIVE_KI = 0.0;
        public static final double SWERVE_MODULE_DRIVE_KD = 0.0;

        public static final double SWERVE_MODULE_DRIVE_KS = 0.125;
        public static final double SWERVE_MODULE_DRIVE_KV = 0.06;
        public static final double SWERVE_MODULE_DRIVE_KA = 0.01;

        // PID constants for simulated swerve modules
        public static final double SIM_SWERVE_MODULE_DRIVE_KP = 0.1;
        public static final double SIM_SWERVE_MODULE_DRIVE_KI = 0.0;
        public static final double SIM_SWERVE_MODULE_DRIVE_KD = 0.0;

        public static final double SIM_SWERVE_MODULE_TURN_KP = 0.5;
        public static final double SIM_SWERVE_MODULE_TURN_KI = 0.0;
        public static final double SIM_SWERVE_MODULE_TURN_KD = 0.0;

        // PID constants for autonomous/pathplanner mode
        public static final double SWERVE_DRIVE_X_KP = 0.2;
        public static final double SWERVE_DRIVE_X_KI = 0.0;
        public static final double SWERVE_DRIVE_X_KD = 0.0;

        public static final double SWERVE_DRIVE_Y_KP = 0.2;
        public static final double SWERVE_DRIVE_Y_KI = 0.0;
        public static final double SWERVE_DRIVE_Y_KD = 0.0;

        public static final double SWERVE_DRIVE_OMEGA_KP = 1.0;
        public static final double SWERVE_DRIVE_OMEGA_KI = 0.0;
        public static final double SWERVE_DRIVE_OMEGA_KD = 0.0;

        // PID constants for simulated autonomous/pathplanner mode
        public static final double SIM_SWERVE_DRIVE_X_KP = 2.0;
        public static final double SIM_SWERVE_DRIVE_X_KI = 0.0;
        public static final double SIM_SWERVE_DRIVE_X_KD = 0.0;

        public static final double SIM_SWERVE_DRIVE_Y_KP = 2.0;
        public static final double SIM_SWERVE_DRIVE_Y_KI = 0.0;
        public static final double SIM_SWERVE_DRIVE_Y_KD = 0.0;

        public static final double SIM_SWERVE_DRIVE_OMEGA_KP = 25.0;
        public static final double SIM_SWERVE_DRIVE_OMEGA_KI = 0.0;
        public static final double SIM_SWERVE_DRIVE_OMEGA_KD = 2.0;

        // PID constants for elevator
        public static final double ELEVATOR_KP = 0.033;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.0;

        public static final double SIM_ELEVATOR_KP = 5000.0;
        public static final double SIM_ELEVATOR_KI = 0.0;
        public static final double SIM_ELEVATOR_KD = 2000.0;

        /**
         * Should only be call by simulation as TunerContants has real values
         */
        public static final double[] getDrivePIDs() {
            return Robot.isReal()
                    ? new double[] { SWERVE_MODULE_DRIVE_KP, SWERVE_MODULE_DRIVE_KI, SWERVE_MODULE_DRIVE_KD,
                            SWERVE_MODULE_DRIVE_KS, SWERVE_MODULE_DRIVE_KV, SWERVE_MODULE_DRIVE_KA }
                    : new double[] { SIM_SWERVE_MODULE_DRIVE_KP, SIM_SWERVE_MODULE_DRIVE_KI,
                            SIM_SWERVE_MODULE_DRIVE_KD };
        }

        /**
         * Should only be call by simulation as TunerContants has real values
         */
        public static final double[] getTurnPIDs() {
            return new double[] { SIM_SWERVE_MODULE_TURN_KP, SIM_SWERVE_MODULE_TURN_KI, SIM_SWERVE_MODULE_TURN_KD };
        }

        /**
         * 
         */
        public static final double[] getDriveXPIDs() {
            return Robot.isReal()
                    ? new double[] { SWERVE_DRIVE_X_KP, SWERVE_DRIVE_X_KI, SWERVE_DRIVE_X_KD }
                    : new double[] { SIM_SWERVE_DRIVE_X_KP, SIM_SWERVE_DRIVE_X_KI, SIM_SWERVE_DRIVE_X_KD };
        }

        /**
         * 
         */
        public static final double[] getDriveYPIDs() {
            return Robot.isReal()
                    ? new double[] { SWERVE_DRIVE_Y_KP, SWERVE_DRIVE_Y_KI, SWERVE_DRIVE_Y_KD }
                    : new double[] { SIM_SWERVE_DRIVE_Y_KP, SIM_SWERVE_DRIVE_Y_KI, SIM_SWERVE_DRIVE_Y_KD };
        }

        /**
         * 
         */
        public static final double[] getDriveOmegaPIDs() {
            return Robot.isReal()
                    ? new double[] { SWERVE_DRIVE_OMEGA_KP, SWERVE_DRIVE_OMEGA_KI, SWERVE_DRIVE_OMEGA_KD }
                    : new double[] { SIM_SWERVE_DRIVE_OMEGA_KP, SIM_SWERVE_DRIVE_OMEGA_KI,
                            SIM_SWERVE_DRIVE_OMEGA_KD };
        }

        /**
         * 
         */
        public static final double[] getElevatorPIDs() {
            return Robot.isReal()
                    ? new double[] { ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD }
                    : new double[] { SIM_ELEVATOR_KP, SIM_ELEVATOR_KI, SIM_ELEVATOR_KD };
        }
    }

    /**
     * Be sure to update SwerveConstants to match robot
     */
    public static final class SwerveConstants {
        /**
         * Manta Raider - 22.0
         * Bealtoven - 18.75
         */
        public static final double trackWidth = Units.inchesToMeters(18.75);
        public static final double wheelBase = Units.inchesToMeters(18.75);
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2, wheelBase / 2);
        public static final double maxLinearSpeed = 4.69;
        public static final double maxLinearAcceleration = 4.0;
        public static final double maxAngularAcceleration = 20.0;
        public static final double maxAngularSpeed = 8.0; // 4.69 / driveBaseRadius;

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) };

        public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

        /**
         * L1 - Manta Raider
         * L3 - Bealtoven
         */
        public static final COTSTalonFXSwerveConstants cotsDriveConstants = MK4i.KrakenX60(MK4i.driveRatios.L3);
        public static final COTSTalonFXSwerveConstants cotsTurnConstants = MK4i.Falcon500(MK4i.driveRatios.L3);
    }

    /**
     * 
     */
    public static final class TeleopConstants {
        public static final double JOYSTICK_AXIS_MODIFIER = 1.0;
        public static final double DEADBAND = 0.1;

        public static final double X_RATE_LIMIT = 6.0;
        public static final double Y_RATE_LIMIT = 6.0;
        public static final double ROTATION_RATE_LIMIT = 5.0 * Math.PI;

        public static final double SPEED_MODIFIER_ONE_HUNDRED = 1.00;
        public static final double SPEED_MODIFIER_THIRTY = 0.30;

        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SwerveConstants.maxLinearSpeed, SwerveConstants.maxLinearAcceleration);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SwerveConstants.maxLinearSpeed, SwerveConstants.maxLinearAcceleration);
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SwerveConstants.maxAngularSpeed, SwerveConstants.maxLinearAcceleration);
    }

    /**
     * 
     */
    public static class VisionConstants {
        public static AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        /**
         * TODO: set camera names and positions - pitch is based on degress from
         * vertical
         */
        public static final Camera ARDUCAM2_CAMERA = new Camera("Arducam_OV9281-2",
                Camera.Type.APRILTAG, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(11.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(9.25),
                0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-30.0));

        public static final Camera ARDUCAM1_CAMERA = new Camera("Arducam_OV9281-1",
                Camera.Type.APRILTAG, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(-11.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(9.25),
                0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-150.0));

        public static final Camera ARDUCAM3_CAMERA = new Camera("Arducam_UC626-2",
                Camera.Type.APRILTAG, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(-10.0), Units.inchesToMeters(-7.5), Units.inchesToMeters(9.0),
                0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-150.0));

        public static final Camera GAME_PIECE_CAMERA = new Camera("WebCam",
                Camera.Type.COLOURED_SHAPE, Camera.Processor.PHOTONVISION, 0,
                Units.inchesToMeters(0.0), 0.0, Units.inchesToMeters(0.0),
                0.0, Units.degreesToRadians(0), 0.0);

        public static final Camera LIMELIGHT3G_CAMERA = new Camera("limelight",
                Camera.Type.APRILTAG, Camera.Processor.LIMELIGHT, 0,
                Units.inchesToMeters(11.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(9.25),
                0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-30.0));

        public static final Camera LIMELIGHT2_CAMERA = new Camera("limelight-two",
                Camera.Type.APRILTAG, Camera.Processor.LIMELIGHT, 0,
                Units.inchesToMeters(10.0), Units.inchesToMeters(-7.0), Units.inchesToMeters(9.0),
                0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-30.0));

        /**
         * TODO: set list of enabled camera
         */
        public static final List<Camera> SIMULATION_CAMERAS = Arrays.asList(ARDUCAM2_CAMERA, ARDUCAM1_CAMERA);
        public static final List<Camera> BEALTOVEN_CAMERAS = Arrays.asList(LIMELIGHT2_CAMERA, ARDUCAM3_CAMERA);
        public static final List<Camera> REEFSCAPE_CAMERAS = Arrays.asList(LIMELIGHT3G_CAMERA, ARDUCAM2_CAMERA);
        public static final List<Camera> CAMERAS = BEALTOVEN_CAMERAS;
        public static final Camera FRONT_CAMERA = CAMERAS.get(0);
        public static final Camera REAR_CAMERA = CAMERAS.get(1);

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.3;
        public static final double APRILTAG_MAX_Z_ERROR = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
        public static double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
        public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data
                                                                                         // available

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] CAMERA_STD_DEV_FACTORS = new double[] {
                1.0, // Camera 0
                1.0 // Camera 1
        };

        // for simulation only
        public static final double DIAGONAL_FOV = 90;
        public static final int IMG_WIDTH = 960;
        public static final int IMG_HEIGHT = 720;
    }
}