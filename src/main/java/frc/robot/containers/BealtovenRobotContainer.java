package frc.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.modules.algae.AlgaeIntakeModuleIOSim;
import frc.robot.modules.coral.CoralIntakeModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIONavX;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveModuleBuilder;

/**
 * 
 */
public class BealtovenRobotContainer extends RobotContainer {
    /**
     * 
     */
    public BealtovenRobotContainer() {
        super();

        SwerveModuleBuilder[] swerveModuleConfigs = getModuleConfigs();

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIONavX(),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[0]),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[1]),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[2]),
                new SwerveModuleIOTalonFX(swerveModuleConfigs[3]));

        this.visionSubsystem = new VisionSubsystem(this.swerveDriveSubsystem,
                this.swerveDriveSubsystem::getPose, this.swerveDriveSubsystem::getRotation);

        this.algaeIntakeSubsystem = new AlgaeIntakeSubsystem(new AlgaeIntakeModuleIOSim());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim(), this.algaeIntakeSubsystem::hasAlgae,
                this.coralIntakeSubsystem::hasCoral);

        registerNamedCommands();
        configureAutoBuilder();
        configureButtonBindings();
    }

    @Override
    public void registerNamedCommands() {
    }

    /**
     * Be sure to update SwerveConstants to match robot
     */
    private SwerveModuleBuilder[] getModuleConfigs() {
        SwerveModuleBuilder frontLeft = new SwerveModuleBuilder(41, 42, 4,
                Rotation2d.fromDegrees(155.566406), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder frontRight = new SwerveModuleBuilder(11, 12, 1,
                Rotation2d.fromDegrees(-317.021484), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backLeft = new SwerveModuleBuilder(31, 32, 3,
                Rotation2d.fromDegrees(-72.861328), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backRight = new SwerveModuleBuilder(21, 22, 2,
                Rotation2d.fromDegrees(-202.763672 + 180), true, true, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);

        return new SwerveModuleBuilder[] { frontLeft, frontRight, backLeft, backRight };
    }

    /**
     * Maple Sim Routines not used for Real Robot
     */
    public void displaySimFieldToAdvantageScope() {
    }

    public void resetSimulationField(Pose2d pose2d) {
    }
}
