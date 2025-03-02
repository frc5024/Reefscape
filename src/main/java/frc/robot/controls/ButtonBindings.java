package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.Elevator.SetElevatorSetpointCmd;
import frc.robot.commands.vision.DriveFromBestTagCommand;
import frc.robot.controls.GameData.CoralPole;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
public class ButtonBindings {
    private final int DRIVER_PORT = 0;
    private final int OPERATOR_PORT = 1;
    private final int TEST_PORT = 2;

    /* Controllers */
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandXboxController testController;

    /* Subsystems */
    Coral coralSubsystem;
    Elevator elevatorSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * 
     */
    public ButtonBindings(SwerveDriveSubsystem swerveDriveSubsystem, Coral coralSubsystem, Elevator elevatorSubsystem,
            VisionSubsystem visionSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        this.driverController = setDriverBindingsController();
        this.operatorController = setOperatorBindingsController();
        this.testController = setTestBindings();
    }

    /**
     * 
     */
    private CommandXboxController setDriverBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(DRIVER_PORT);

        // Toggle game piece modes
        commandXboxController.back()
                .whileTrue(runOnce(() -> GameData.getInstance().toggleGamePieceMode()));

        // switch from robot relative to field relative
        commandXboxController.start()
                .whileTrue(either(
                        runOnce(this.swerveDriveSubsystem::disableFieldRelative,
                                this.swerveDriveSubsystem),
                        runOnce(this.swerveDriveSubsystem::enableFieldRelative,
                                this.swerveDriveSubsystem),
                        this.swerveDriveSubsystem::isFieldRelative));

        commandXboxController.x().onTrue(new InstantCommand(() -> this.swerveDriveSubsystem.zeroHeading()));

        commandXboxController.y().onTrue(this.coralSubsystem.outtakeCommand());

        commandXboxController.b().whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem,
                Constants.ElevatorContants.rootPosition));

        // commandXboxController.a().onTrue(new InstantCommand(() ->
        // toggleVisionMode()));

        // commandXboxController.leftBumper().onTrue(new InstantCommand(() ->
        // s_Swerve.isSlowMode =
        // true));
        // commandXboxController.leftBumper().onFalse(new InstantCommand(() ->
        // s_Swerve.isSlowMode =
        // false));

        // two intake commands
        commandXboxController.rightBumper().whileTrue(this.coralSubsystem.intakeCommand());
        commandXboxController.rightBumper().whileTrue(new SetElevatorSetpointCmd(elevatorSubsystem,
                Constants.ElevatorContants.rootPosition));

        // Drive to right pole of best apriltag
        commandXboxController.rightTrigger()
                .whileTrue(new DriveFromBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose,
                        false,
                        GameData.getInstance().getGamePieceMode()));

        // Drive to left pole of best apriltag
        commandXboxController.leftTrigger()
                .whileTrue(new DriveFromBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose,
                        true,
                        GameData.getInstance().getGamePieceMode()));

        // Drive to selected reef station
        // commandXboxController.rightBumper()
        // .whileTrue(new DriveReefStationCommand(this.swerveDriveSubsystem,
        // this.swerveDriveSubsystem::getPose,
        // GameData.getInstance()::getReefStationIndex,
        // GameData.getInstance().getCoralPole(),
        // GameData.getInstance().getGamePieceMode()));

        // // Drive to selected reef station
        // commandXboxController.leftBumper()
        // .whileTrue(new DriveReefStationCommand(this.swerveDriveSubsystem,
        // this.swerveDriveSubsystem::getPose,
        // GameData.getInstance()::getReefStationIndex,
        // GameData.getInstance().getCoralPole(),
        // GameData.getInstance().getGamePieceMode()));

        // Set reef position
        commandXboxController.povUp().onTrue(runOnce(() -> GameData.getInstance().setReefStationIndex(1)));
        commandXboxController.povDown().onTrue(runOnce(() -> GameData.getInstance().setReefStationIndex(-1)));
        commandXboxController.povLeft().onTrue(runOnce(() -> GameData.getInstance().setCoralPole(CoralPole.LEFT)));
        commandXboxController.povRight().onTrue(runOnce(() -> GameData.getInstance().setCoralPole(CoralPole.RIGHT)));

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setOperatorBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(OPERATOR_PORT);

        // (operator)
        // potential binding fix
        // operator.b().onTrue(new ConditionalCommand(getAutonomousCommand(),
        // getAutonomousCommand(), () -> visionMode));

        // operator.a().onTrue(new SetElevatorSetpointCmd(elevatorSubsystem,
        // Constants.elevatorConstants.zeroPosition));

        commandXboxController.povLeft()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, Constants.ElevatorContants.L1Position));
        commandXboxController.povDown()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, Constants.ElevatorContants.L2Position));
        commandXboxController.povRight()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, Constants.ElevatorContants.L3position));
        commandXboxController.povUp()
                .whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem, Constants.ElevatorContants.L4position));

        commandXboxController.a().whileTrue(new SetElevatorSetpointCmd(this.elevatorSubsystem,
                Constants.ElevatorContants.rootPosition));

        commandXboxController.rightTrigger().onTrue(this.coralSubsystem.lowerRampCommand());
        // operator.rightTrigger().onTrue(extendClimb());

        commandXboxController.b().onTrue(coralSubsystem.lowerRampCommand());

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setTestBindings() {
        CommandXboxController commandXboxController = new CommandXboxController(TEST_PORT);

        return commandXboxController;
    }

    /**
     * 
     */
    public CommandXboxController getDriverController() {
        return this.driverController;
    }

    public CommandXboxController getOperatorController() {
        return this.operatorController;
    }

    public CommandXboxController getTestController() {
        return this.testController;
    }
}
