package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.vision.GoToSetPositionPerTagCmd;
import frc.robot.commands.vision.autoSetPositionTagID;
import frc.robot.commands.vision.isPathRun;
import frc.robot.modules.gyro.GyroModuleIONavX;
import frc.robot.modules.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
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

        this.visionSubsystem = new VisionSubsystem(VisionConstants.MANTARAIDER_CAMERAS, this.swerveDriveSubsystem,
                this.swerveDriveSubsystem::getPose, this.swerveDriveSubsystem::getRotation);

        // this.algaeSubsystem = new AlgaeSubsystem(new AlgaeModuleIOSim());
        // this.climbSubsystem = new ClimbSubsystem(new ClimbModuleIOTalonFX());
        // this.coralSubsystem = new CoralSubsystem(new CoralModuleIOSparkFlex());
        // this.elevatorSubsystem = new ElevatorSubsystem(new
        // ElevatorModuleIOSparkMax(), this.algaeSubsystem::hasAlgae,
        // this.coralSubsystem::hasCoral);

        this.climb = Climb.getInstance();
        this.coral = Coral.getInstance();
        this.elevator = Elevator.getInstance();
        this.rumble = Rumble.getInstance();
        this.limelightSubsystem = Limelight.getInstance();

        registerNamedCommands();
        configureAutoBuilder();
        configureBindings();
    }

    @Override
    protected void configureAutoBuilder() {
        this.autonomousChooser.addOption("Non-Processor side 2/3 piece (COMPLETE)",
                Commands.sequence(new PathPlannerAuto("Start 11R"), new PathPlannerAuto("11R TS 6R"),
                        new PathPlannerAuto("6R TS 6L")));

        this.autonomousChooser.addOption("Processor side 2/3 piece (COMPLETE)",
                Commands.sequence(new PathPlannerAuto("Start 9R"), new PathPlannerAuto("9R BS 8"),
                        new PathPlannerAuto("8R BS 8L")));

        this.autonomousChooser.addOption("Middle 1 piece right (COMPLETE)", new PathPlannerAuto("MiddleRight"));
        this.autonomousChooser.addOption("Middle 1 piece left (COMPLETE)", new PathPlannerAuto("MiddleLeft"));

        this.autonomousChooser.addOption("Testing Elevatoring", new PathPlannerAuto("Start 11R"));
    }

    @Override
    public void registerNamedCommands() {
        NamedCommands.registerCommand("DriveRightTag",
                new GoToSetPositionPerTagCmd(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_RIGHT_OFFSET));

        NamedCommands.registerCommand("DriveLeftTag",
                new GoToSetPositionPerTagCmd(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_LEFT_OFFSET));

        NamedCommands.registerCommand("DriveLeftTag11",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_LEFT_OFFSET, 11));

        NamedCommands.registerCommand("DriveRightTag11",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_RIGHT_OFFSET, 11));

        NamedCommands.registerCommand("DriveRightTag10",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_RIGHT_OFFSET, 10));

        NamedCommands.registerCommand("DriveLeftTag10",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_LEFT_OFFSET, 10));

        NamedCommands.registerCommand("DriveLeftTag9",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_LEFT_OFFSET, 9));

        NamedCommands.registerCommand("DriveRightTag9",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_RIGHT_OFFSET, 9));

        NamedCommands.registerCommand("DriveRightTag8",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_RIGHT_OFFSET, 8));

        NamedCommands.registerCommand("DriveLeftTag8",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_LEFT_OFFSET, 8));

        NamedCommands.registerCommand("DriveRightTag6",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_RIGHT_OFFSET, 6));

        NamedCommands.registerCommand("DriveLeftTag6",
                new autoSetPositionTagID(limelightSubsystem, this.swerveDriveSubsystem,
                        FieldConstants.REEF_POLE_LEFT_OFFSET, 6));

        NamedCommands.registerCommand("elevatorMode", this.elevator.goToModePosition());

        // Coral
        NamedCommands.registerCommand("ScoreCoral", this.coral.outtakeAutoCommand());
        NamedCommands.registerCommand("IntakeCoral", this.coral.intakeCommand());

        NamedCommands.registerCommand("Confirm Vision", new InstantCommand(() -> limelightSubsystem.pathIsDone(true))); // fix
        // better
        NamedCommands.registerCommand("Wait For Vision", new isPathRun(limelightSubsystem));

        // Elevator
        NamedCommands.registerCommand("L4", this.elevator.goToL4Position());
        NamedCommands.registerCommand("L3", this.elevator.goToL3Position());
        NamedCommands.registerCommand("L2", this.elevator.goToL2Position());
        NamedCommands.registerCommand("L1", this.elevator.goToL1Position());
        NamedCommands.registerCommand("ElevatorRoot", this.elevator.bottomAutoElevator());

        /* */
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
