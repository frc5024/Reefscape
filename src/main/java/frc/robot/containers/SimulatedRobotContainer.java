package frc.robot.containers;

import frc.robot.generated.TunerConstants;
import frc.robot.modules.gyro.GyroIO;
import frc.robot.modules.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class SimulatedRobotContainer extends RobotContainer {
    public SimulatedRobotContainer() {
        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroIO() {
                },
                new SwerveModuleIOSim(TunerConstants.FrontLeft),
                new SwerveModuleIOSim(TunerConstants.FrontRight),
                new SwerveModuleIOSim(TunerConstants.BackLeft),
                new SwerveModuleIOSim(TunerConstants.BackRight));

        // Configure the button bindings
        configureButtonBindings();
    }
}
