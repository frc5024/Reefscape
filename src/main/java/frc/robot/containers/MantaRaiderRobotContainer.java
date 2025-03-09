package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.vision.DriveFromBestTagCommand;
import frc.robot.controls.GameData;
import frc.robot.modules.gyro.GyroModuleIONavX;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SwerveModuleBuilder;

/**
 * 
 */
public class MantaRaiderRobotContainer extends RobotContainer {
    /**
     * 
     */
    public MantaRaiderRobotContainer() {
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

        // this.algaeSubsystem = new AlgaeSubsystem(new AlgaeModuleIOSim());
        // this.climbSubsystem = new ClimbSubsystem(new ClimbModuleIOTalonFX());
        // this.coralSubsystem = new CoralSubsystem(new CoralModuleIOSparkFlex());
        // this.elevatorSubsystem = new ElevatorSubsystem(new
        // ElevatorModuleIOSparkMax(), this.algaeSubsystem::hasAlgae,
        // this.coralSubsystem::hasCoral);

        this.climb = Climb.getInstance();
        this.coral = new Coral();
        this.elevator = Elevator.getInstance();
        this.rumble = Rumble.getInstance();
        this.limelightSubsystem = new Limelight();
        this.lEDs = LEDs.getInstance();

        registerNamedCommands();
        configureAutoBuilder();
        configureBindings();
    }

    @Override
    public void registerNamedCommands() {
        NamedCommands.registerCommand("ScoreCoral", new InstantCommand(() -> {
            this.coralSubsystem.addAction(CoralSubsystem.Action.EJECT);
        }));
        NamedCommands.registerCommand("IntakeCoral", new InstantCommand(() -> {
            this.coralSubsystem.addAction(CoralSubsystem.Action.INTAKE);
        }));
        NamedCommands.registerCommand("DriveRightTag",
                new DriveFromBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, false, GameData.getInstance().getGamePieceMode()));
        NamedCommands.registerCommand("DriveLeftTag",
                new DriveFromBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, true, GameData.getInstance().getGamePieceMode()));

        NamedCommands.registerCommand("ElevatorL0", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_BOTTOM);
        }));
        NamedCommands.registerCommand("ElevatorL2", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_2);
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
     * Be sure to update SwerveConstants to match robot
     */
    private SwerveModuleBuilder[] getModuleConfigs() {
        SwerveModuleBuilder frontLeft = new SwerveModuleBuilder(41, 42, 4,
                Rotation2d.fromDegrees(52.8), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder frontRight = new SwerveModuleBuilder(11, 12, 1,
                Rotation2d.fromDegrees(92.9 + 180), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backLeft = new SwerveModuleBuilder(31, 32, 3,
                Rotation2d.fromDegrees(-60), true, false, SwerveConstants.cotsDriveConstants,
                SwerveConstants.cotsTurnConstants);
        SwerveModuleBuilder backRight = new SwerveModuleBuilder(21, 22, 2,
                Rotation2d.fromDegrees(-35.7), true, true, SwerveConstants.cotsDriveConstants,
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
