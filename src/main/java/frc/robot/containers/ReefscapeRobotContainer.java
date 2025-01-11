package frc.robot.containers;

import frc.robot.generated.TunerConstants;
import frc.robot.modules.gyro.GyroIOPigeon2;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class ReefscapeRobotContainer extends RobotContainer {
    /**
     * 
     */
    public ReefscapeRobotContainer() {
        super();

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new SwerveModuleIOTalonFX(TunerConstants.FrontLeft),
                new SwerveModuleIOTalonFX(TunerConstants.FrontRight),
                new SwerveModuleIOTalonFX(TunerConstants.BackLeft),
                new SwerveModuleIOTalonFX(TunerConstants.BackRight));

        // Configure the button bindings
        configureButtonBindings();
    }
}
