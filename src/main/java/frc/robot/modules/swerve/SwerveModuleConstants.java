package frc.robot.modules.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.COTSTalonFXSwerveConstants;
import frc.robot.utils.COTSTalonFXSwerveConstants.SDS.MK4i;
import frc.robot.utils.SwerveModuleBuilder;

public class SwerveModuleConstants {
    public static final double trackWidth = Units.inchesToMeters(18.75);
    public static final double wheelBase = Units.inchesToMeters(18.75);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2, wheelBase / 2);
    public static final double maxLinearSpeed = 4.69;
    public static final double maxAngularSpeed = 4.69 / driveBaseRadius;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) };

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    public static final COTSTalonFXSwerveConstants cotsDriveConstants = MK4i.Falcon500(MK4i.driveRatios.L3);
    public static final COTSTalonFXSwerveConstants cotsTurnConstants = MK4i.Falcon500(MK4i.driveRatios.L3);

    private static SwerveModuleBuilder frontLeft = new SwerveModuleBuilder(41, 42, 4,
            Rotation2d.fromRotations(-0.443604), false, false, cotsDriveConstants,
            cotsTurnConstants);
    private static SwerveModuleBuilder frontRight = new SwerveModuleBuilder(11, 12, 1,
            Rotation2d.fromRotations(-0.118896), false, false, cotsDriveConstants,
            cotsTurnConstants);
    private static SwerveModuleBuilder backLeft = new SwerveModuleBuilder(31, 32, 3,
            Rotation2d.fromRotations(0.196533), false, false, cotsDriveConstants,
            cotsTurnConstants);
    private static SwerveModuleBuilder backRight = new SwerveModuleBuilder(21, 22, 2,
            Rotation2d.fromRotations(0.054443), false, false, cotsDriveConstants,
            cotsTurnConstants);

    public static final SwerveModuleBuilder[] swerveModuleConfigs = { frontLeft, frontRight, backLeft, backRight };
}
