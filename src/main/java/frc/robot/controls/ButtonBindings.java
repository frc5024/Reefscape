package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.vision.DriveFromBestTagCommand;
import frc.robot.commands.vision.DriveNearestCoralStationCommand;
import frc.robot.commands.vision.DriveProcessorCommand;
import frc.robot.commands.vision.DriveReefStationCommand;
import frc.robot.controls.GameData.CoralPole;
import frc.robot.controls.GameData.GamePieceMode;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * 
     */
    public ButtonBindings(SwerveDriveSubsystem swerveDriveSubsystem, AlgaeSubsystem algaeSubsystem,
            CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.algaeSubsystem = algaeSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.visionSubsystem = visionSubsystem;

        this.driverController = setDriverBindingsController();
        this.operatorController = setOperatorBindingsController();

        // Set this to whichever button bindings you want to test
        // this.buttonTestController = setLEDTestBindingsController();
        this.testController = setTuningBindings();
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

        // Lock to 0° when A button is held
        // commandXboxController.a()
        // .whileTrue(
        // SwerveDriveCommands.joystickDriveAtAngle(
        // swerveDriveSubsystem,
        // () -> -commandXboxController.getLeftY(),
        // () -> -commandXboxController.getLeftX(),
        // () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(swerveDriveSubsystem::stopWithX,
        // swerveDriveSubsystem));

        commandXboxController.a().onTrue(new InstantCommand(() -> this.swerveDriveSubsystem.zeroHeading()));

        // Set robot pose based on best vision target
        // commandXboxController.a()
        // .onTrue(runOnce(() -> this.swerveDriveSubsystem.setPose(
        // new Pose2d(this.visionSubsystem.getBestRobotPose().getTranslation(), new
        // Rotation2d())),
        // this.swerveDriveSubsystem)
        // .ignoringDisable(true));

        // Reset gyro to 0° when B button is pressed
        commandXboxController.b()
                .onTrue(runOnce(() -> this.swerveDriveSubsystem.setPose(
                        new Pose2d(this.swerveDriveSubsystem.getPose().getTranslation(), new Rotation2d())),
                        this.swerveDriveSubsystem)
                        .ignoringDisable(true));

        // Drive to nearest coral station
        commandXboxController.x()
                .whileTrue(new DriveNearestCoralStationCommand(this.swerveDriveSubsystem));

        // Drive to processor station
        commandXboxController.y()
                .whileTrue(new DriveProcessorCommand(this.swerveDriveSubsystem));

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
        commandXboxController.rightBumper()
                .whileTrue(new DriveReefStationCommand(this.swerveDriveSubsystem,
                        this.swerveDriveSubsystem::getPose,
                        GameData.getInstance()::getReefStationIndex,
                        GameData.getInstance().getCoralPole(),
                        GameData.getInstance().getGamePieceMode()));

        // Drive to selected reef station
        commandXboxController.leftBumper()
                .whileTrue(new DriveReefStationCommand(this.swerveDriveSubsystem,
                        this.swerveDriveSubsystem::getPose,
                        GameData.getInstance()::getReefStationIndex,
                        GameData.getInstance().getCoralPole(),
                        GameData.getInstance().getGamePieceMode()));

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

        // Toggle game piece modes
        commandXboxController.back()
                .whileTrue(runOnce(() -> GameData.getInstance().toggleGamePieceMode()));

        commandXboxController.a()
                .whileTrue(runOnce(() -> this.elevatorSubsystem
                        .addAction(ElevatorSubsystem.Action.MOVE_TO_BOTTOM)));
        commandXboxController.x()
                .whileTrue(runOnce(() -> this.elevatorSubsystem
                        .addAction(GameData.getInstance().getGamePieceMode().get() == GamePieceMode.ALGAE
                                ? ElevatorSubsystem.Action.MOVE_TO_ALGAE_1
                                : ElevatorSubsystem.Action.MOVE_TO_CORAL_2)));
        commandXboxController.b()
                .whileTrue(runOnce(() -> this.elevatorSubsystem
                        .addAction(GameData.getInstance().getGamePieceMode().get() == GamePieceMode.ALGAE
                                ? ElevatorSubsystem.Action.MOVE_TO_ALGAE_2
                                : ElevatorSubsystem.Action.MOVE_TO_CORAL_3)));
        commandXboxController.y()
                .whileTrue(runOnce(() -> this.elevatorSubsystem
                        .addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_4)));

        commandXboxController.leftTrigger()
                .whileTrue(runOnce(() -> {
                    if (GameData.getInstance().getGamePieceMode().get() == GamePieceMode.ALGAE) {
                        this.algaeSubsystem.addAction(AlgaeSubsystem.Action.EJECT);
                    } else {
                        this.coralSubsystem.addAction(CoralSubsystem.Action.EJECT);
                    }
                }));

        commandXboxController.rightTrigger()
                .whileTrue(runOnce(() -> {
                    if (GameData.getInstance().getGamePieceMode().get() == GamePieceMode.ALGAE) {
                        this.algaeSubsystem.addAction(AlgaeSubsystem.Action.INTAKE);
                    } else {
                        this.coralSubsystem.addAction(CoralSubsystem.Action.INTAKE);
                    }
                }));

        // commandXboxController.leftBumper()
        // .whileTrue(runOnce(() -> this.coralSubsystem
        // .addAction(CoralSubsystem.Action.EJECT)));

        // commandXboxController.rightBumper()
        // .whileTrue(runOnce(() -> this.coralSubsystem
        // .addAction(CoralSubsystem.Action.INTAKE)));

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setLEDTestBindingsController() {
        CommandXboxController commandXboxController = new CommandXboxController(TEST_PORT);

        commandXboxController.a()
                .whileTrue(startEnd(() -> LEDSubsystem.getInstance().solidGreen(),
                        () -> LEDSubsystem.getInstance().solidBlack()));
        commandXboxController.x()
                .whileTrue(startEnd(() -> LEDSubsystem.getInstance().solidBlue(),
                        () -> LEDSubsystem.getInstance().solidBlack()));
        commandXboxController.b()
                .whileTrue(startEnd(() -> LEDSubsystem.getInstance().solidRed(),
                        () -> LEDSubsystem.getInstance().solidBlack()));
        commandXboxController.y()
                .whileTrue(startEnd(() -> LEDSubsystem.getInstance().solidYellow(),
                        () -> LEDSubsystem.getInstance().solidBlack()));

        return commandXboxController;
    }

    /**
     * 
     */
    private CommandXboxController setTuningBindings() {
        CommandXboxController commandXboxController = new CommandXboxController(TEST_PORT);

        /* Bindings for drivetrain characterization */
        /*
         * These bindings require multiple buttons pushed to swap between quastatic and
         * dynamic
         */
        /*
         * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
         */
        commandXboxController.back().and(commandXboxController.y())
                .whileTrue(this.swerveDriveSubsystem.sysIdDynamic(Direction.kForward));
        commandXboxController.back().and(commandXboxController.x())
                .whileTrue(this.swerveDriveSubsystem.sysIdDynamic(Direction.kReverse));
        commandXboxController.start().and(commandXboxController.y())
                .whileTrue(this.swerveDriveSubsystem.sysIdQuasistatic(Direction.kForward));
        commandXboxController.start().and(commandXboxController.x())
                .whileTrue(this.swerveDriveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // Toggle game piece modes
        commandXboxController.back()
                .whileTrue(runOnce(() -> GameData.getInstance().toggleGamePieceMode()));

        // Set robot pose based on best vision target
        commandXboxController.b()
                .onTrue(runOnce(() -> this.swerveDriveSubsystem.setPose(
                        new Pose2d(this.visionSubsystem.getBestRobotPose().getTranslation(), new Rotation2d())),
                        this.swerveDriveSubsystem)
                        .ignoringDisable(true));

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
