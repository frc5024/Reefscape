package frc.robot.containers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.TunerConstants;
import frc.robot.modules.algae.AlgaeIntakeModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIOSim;
import frc.robot.modules.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
                new GyroModuleIOSim(null),
                new SwerveModuleIOSim(TunerConstants.FrontLeft),
                new SwerveModuleIOSim(TunerConstants.FrontRight),
                new SwerveModuleIOSim(TunerConstants.BackLeft),
                new SwerveModuleIOSim(TunerConstants.BackRight));

        this.visionSubsystem = new VisionSubsystem(this.swerveDriveSubsystem,
                this.swerveDriveSubsystem::getPose, this.swerveDriveSubsystem::getRotation);

        this.algaeIntakeSubsystem = new AlgaeIntakeSubsystem(new AlgaeIntakeModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim());

        registerNamedCommands();
        configureAutoBuilder();
        configureButtonBindings();

        // Initiate the LEDSubsystem
        LEDSubsystem.getInstance();

    }

    @Override
    public void registerNamedCommands() {
    }

    @Override
    public void displaySimFieldToAdvantageScope() {
    }

    @Override
    public void resetSimulationField(Pose2d pose2d) {
    }
}
