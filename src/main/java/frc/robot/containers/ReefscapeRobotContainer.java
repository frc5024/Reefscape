package frc.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.TunerConstants;
import frc.robot.modules.gyro.GyroModuleIONavX;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
                new GyroModuleIONavX(),
                new SwerveModuleIOTalonFX(TunerConstants.FrontLeft),
                new SwerveModuleIOTalonFX(TunerConstants.FrontRight),
                new SwerveModuleIOTalonFX(TunerConstants.BackLeft),
                new SwerveModuleIOTalonFX(TunerConstants.BackRight));

        this.visionSubsystem = new VisionSubsystem(this.swerveDriveSubsystem,
                this.swerveDriveSubsystem::getPose,
                this.swerveDriveSubsystem::getRotation);

        // this.algaeIntakeSubsystem = new AlgaeIntakeSubsystem(new
        // AlgaeintakeModuleIOSparkMax());
        // this.coralIntakeSubsystem = new CoralIntakeSubsystem(new
        // CoralintakeModuleIOSparkFlex());
        // this.elevatorSubsystem = new ElevatorSubsystem(new
        // ElevatorModuleIOSparkMax(), this.algaeIntakeSubsystem::hasAlgae(),
        // this.coralIntakeSubsystem::hasCoral());

        registerNamedCommands();
        // configureAutoBuilder();
        configureButtonBindings();
    }

    @Override
    public void registerNamedCommands() {
    }

    /**
     * Maple Sim Routines not used for Real Robot
     */
    public void displaySimFieldToAdvantageScope() {
    }

    public void resetSimulationField(Pose2d pose2d) {
    }
}
