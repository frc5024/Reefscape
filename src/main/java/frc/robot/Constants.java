package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.leds.ILEDPreset;
import frc.lib.leds.LEDPreset;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static class LEDs {
        public final static int ledPort = 9; // Port for LED, Make sure it is PWM not DIO
        public final static ILEDPreset defaultLED = LEDPreset.Solid.kRed;// Default Colour
    }

    public static class Servo {
        public final static int servoPort = 8;// Port for Servo, Make sure it is PWM not DIO
    }

    // AdvantageKit simulation
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        REAL, // Running on a real robot
        SIM, // Running a physics simulator
        REPLAY // Replaying from a log file
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final double stickDeadband = 0.1;

    public static final class Vision {
        // arbitrary scale
        public static final double rightOffset = 0.1502;
        public static final double leftOffset = -0.169025;
        public static final double noOffset = 0;

        // changes smaller adjustments and max speed
        public static final double rotationPIDMultiplier = 1.7;
        public static final double strafePIDMultiplier = 2.3;
        public static final double distancePIDMultiplier = 2;

        // arbitrary scale // limits the PID output creating a capped speed
        public static final double strafePIDCap = 0.15;
        public static final double distancePIDCap = 0.3;

        public static final double rotationTolerance = 1.6; // degrees
        public static final double strafeTolerance = centimetersToMeters(2);
        public static final double distanceTolerance = centimetersToMeters(3); // to be adjusted

        public static double centimetersToMeters(double Centimeters) {
            return Centimeters / 100;
        }
    }

    public static class LEDsConstants { // changed name to LEDsConstants
        public final static int ledPort = 0; // Port for LED, Make sure it is PWM not DIO
        public final static ILEDPreset defaultLED = LEDPreset.Solid.kGold;// Default Colour

    }

    public static class elevatorConstants {
        // elevator values
        public static final double elevatorMaxSpeed = 75;
        public static final double elevatorMaxAccel = 75;
        public static final double zeroPosition = 0;
        public static final int motorID1 = 60;
        public static final int motorID2 = 61;

        // values for PID
        public static final double kP = 0.051;
        public static final double kI = 0;
        public static final double kD = 0;

        // values for feed forward
        public static final double kV = 0.1;
        public static final double kA = 0.05;

        public static final double G = 0.5;
        public static final double minimumBottomValue = 2.5; // encoder value will not always be zero so we create a
                                                             // tolerance value

        // public static final double radianstoCM(double CM) {
        // double radians;
        // radians = Units.degreesToRadians(CM/1.4765*360);
        // return(radians);
        // }

        // one rotation equals 1.4765 cm

        // position constants for the different levels to score in rotations
        public static final double rootPosition = -10;
        public static final double rootAutoPosition = 0;
        public static final double L1Position = 14;
        public static final double L2Position = (17.0802 - 1) + 1;
        public static final double Algae1 = 5.97514;
        public static final double L3Position = (31.37261 - 1.5) + 1;
        public static final double Algae2 = 8.37068;
        public static final double L4Position = 50.57406 + 1;

    }

    // constants for intake and channels
    public static final class coralConstants {
        public static final int coralMotorChannel = 51;
        public static final int coralMotorReversedChannel = 52;
        public static final int linebreakChannel = 0;
        public static final int servoChannel = 0;

        public static double intakeSpeed = -0.1;
        public static double outtakeSpeed = -0.10;
        public static double outtakeL4Speed = -0.15;
        public static double outtakeAutoSpeed = -0.10;

        public static double L1Speed = -0.20;
        public static double plopSpeed = -0.35;
        public static double rampSpeed = 0.1;

        // public static double servoRotate = 0.5;
        // public static double servoReset = -0.5;
    }

    public static final class AlgaeConstant {
        public static double algaeSpeed = 0.3;
        // public static double algaeInPos = 1;
        // public static double algaeOutPos = 0;
    }

    public static final class Swerve {
        public static final int AHRS = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        public static final COTSTalonFXSwerveConstants chosenDriveModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5); // TODO: This must be tuned tos pecific
                                                                            // robot
        public static final double wheelBase = Units.inchesToMeters(18.5); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenDriveModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenDriveModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (0.32); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51);
        public static final double driveKA = (0.27);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.0; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 9.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52.8); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.9 + 180); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-60); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-35.7); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public final class ClimbConstants {

        public static final int climbMotorID = 7;

        public static final double startPosition = -6.00;

        public static final double extendedPosition = 65; // out encoder

        public static final double climbSpeed = -0.7; // motor speeds
        public static final double extendSpeed = 0.4;

    }
}