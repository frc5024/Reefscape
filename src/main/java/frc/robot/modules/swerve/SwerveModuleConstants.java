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
    public static final double maxLinearSpeed = 2.0; // 4.69;
    public static final double maxAcceleration = 2.0; // 4.0;
    public static final double maxAngularSpeed = 2.0 / driveBaseRadius; // 4.69 / driveBaseRadius;

    public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) };

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    public static final COTSTalonFXSwerveConstants cotsDriveConstants = MK4i.Falcon500(MK4i.driveRatios.L3);
    public static final COTSTalonFXSwerveConstants cotsTurnConstants = MK4i.Falcon500(MK4i.driveRatios.L3);

    private static SwerveModuleBuilder frontLeft = new SwerveModuleBuilder(41, 42, 4,
            Rotation2d.fromDegrees(155.566406), true, false, cotsDriveConstants,
            cotsTurnConstants);
    private static SwerveModuleBuilder frontRight = new SwerveModuleBuilder(11, 12, 1,
            Rotation2d.fromDegrees(-317.021484), true, false, cotsDriveConstants,
            cotsTurnConstants);
    private static SwerveModuleBuilder backLeft = new SwerveModuleBuilder(31, 32, 3,
            Rotation2d.fromDegrees(-72.861328), true, false, cotsDriveConstants,
            cotsTurnConstants);
    private static SwerveModuleBuilder backRight = new SwerveModuleBuilder(21, 22, 2,
            Rotation2d.fromDegrees(-202.763672 + 180), true, true, cotsDriveConstants,
            cotsTurnConstants);

    public static final SwerveModuleBuilder[] swerveModuleConfigs = { frontLeft, frontRight, backLeft, backRight };
}
