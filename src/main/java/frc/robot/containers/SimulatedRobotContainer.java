package frc.robot.containers;

import frc.robot.generated.TunerConstants;
import frc.robot.modules.gyro.GyroModuleIO;
import frc.robot.modules.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class SimulatedRobotContainer extends RobotContainer {
    /**
     * 
     */
    public SimulatedRobotContainer() {
        super();

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIO() {
                },
                new SwerveModuleIOSim(TunerConstants.FrontLeft),
                new SwerveModuleIOSim(TunerConstants.FrontRight),
                new SwerveModuleIOSim(TunerConstants.BackLeft),
                new SwerveModuleIOSim(TunerConstants.BackRight));

        configureAutoBuilder();
        configureButtonBindings();
    }
}
