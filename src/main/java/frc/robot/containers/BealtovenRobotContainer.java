package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.modules.algae.AlgaeModuleIOSim;
import frc.robot.modules.coral.CoralModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIOPigeon2;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
public class BealtovenRobotContainer extends RobotContainer {
    /**
     * 
     */
    public BealtovenRobotContainer() {
        super();

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIOPigeon2(),
                new SwerveModuleIOTalonFX(TunerConstants.FrontLeft),
                new SwerveModuleIOTalonFX(TunerConstants.FrontRight),
                new SwerveModuleIOTalonFX(TunerConstants.BackLeft),
                new SwerveModuleIOTalonFX(TunerConstants.BackRight));

        this.visionSubsystem = new VisionSubsystem(VisionConstants.BEALTOVEN_CAMERAS, this.swerveDriveSubsystem,
                this.swerveDriveSubsystem::getPose, this.swerveDriveSubsystem::getRotation);

        this.algaeSubsystem = new AlgaeSubsystem(new AlgaeModuleIOSim());
        this.coralSubsystem = new CoralSubsystem(new CoralModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim(), this.algaeSubsystem::hasAlgae,
                this.coralSubsystem::hasCoral);

        registerNamedCommands();
        configureAutoBuilder();
        configureButtonBindings();
    }

    @Override
    public void registerNamedCommands() {
        NamedCommands.registerCommand("ElevatorL0", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_BOTTOM);
        }));
        NamedCommands.registerCommand("ElevatorL4", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_4);
        }));
        NamedCommands.registerCommand("EjectCoral", new InstantCommand(() -> {
            this.coralSubsystem.addAction(CoralSubsystem.Action.EJECT);
        }));
        NamedCommands.registerCommand("IntakeCoral", new InstantCommand(() -> {
            this.coralSubsystem.addAction(CoralSubsystem.Action.INTAKE);
        }));
        NamedCommands.registerCommand("WaitForEject", new WaitUntilCommand(this.coralSubsystem::hasEjected));
        NamedCommands.registerCommand("WaitForElevator", new WaitUntilCommand(this.elevatorSubsystem::atGoal));
        NamedCommands.registerCommand("WaitForIntake", new WaitUntilCommand(this.coralSubsystem::hasCoral));
    }

    /**
     * Maple Sim Routines not used for Real Robot
     */
    public void displaySimFieldToAdvantageScope() {
    }

    public void resetSimulationField(Pose2d pose2d) {
    }
}
