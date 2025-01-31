package frc.robot.controls;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveNearestCoralStationCommand;
import frc.robot.commands.DriveProcessorCommand;
import frc.robot.commands.DriveToBestTagCommand;
import frc.robot.commands.DriveToReefStationCommand;
import frc.robot.controls.GameData.CoralPole;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Action;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
public class ButtonBindings {
    /* Controllers */
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * 
     */
    public ButtonBindings(SwerveDriveSubsystem swerveDriveSubsystem, AlgaeIntakeSubsystem algaeIntakeSubsystem,
            ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
        this.driverController = new CommandXboxController(0);
        this.operatorController = new CommandXboxController(1);

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.visionSubsystem = visionSubsystem;

        configureDriverBindings();
        configureOperatorBindings();
    }

    /**
     * 
     */
    private void configureDriverBindings() {
        // Toggle game piece modes
        this.driverController.back()
                .whileTrue(runOnce(() -> GameData.getInstance().toggleDriveMode()));

        // Lock to 0° when A button is held
        // this.driverController.a()
        // .whileTrue(
        // SwerveDriveCommands.joystickDriveAtAngle(
        // swerveDriveSubsystem,
        // () -> -controller.getLeftY(),
        // () -> -controller.getLeftX(),
        // () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(swerveDriveSubsystem::stopWithX,
        // swerveDriveSubsystem));

        // Reset gyro to 0° when B button is pressed
        this.driverController.b()
                .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(
                        new Pose2d(swerveDriveSubsystem.getPose().getTranslation(), new Rotation2d())),
                        swerveDriveSubsystem)
                        .ignoringDisable(true));

        // Drive to nearest coral station
        this.driverController.x()
                .whileTrue(new DriveNearestCoralStationCommand(this.swerveDriveSubsystem));

        // Drive to processor station
        this.driverController.y()
                .whileTrue(new DriveProcessorCommand(this.swerveDriveSubsystem));

        // Drive to selected reef station
        this.driverController.rightTrigger()
                .whileTrue(new DriveToReefStationCommand(this.swerveDriveSubsystem, this.swerveDriveSubsystem::getPose,
                        GameData.getInstance()::getReefStationIndex, GameData.getInstance().getCoralPole(),
                        GameData.getInstance().getDriveMode()));

        // Drive to right pole of best apriltag
        this.driverController.rightBumper()
                .whileTrue(new DriveToBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, VisionConstants.DATA_FROM_CAMERA,
                        GameData.getInstance().getCoralPole(),
                        GameData.getInstance().getDriveMode()));

        // Drive to left pole of best apriltag
        this.driverController.leftBumper()
                .whileTrue(new DriveToBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, VisionConstants.DATA_FROM_CAMERA,
                        GameData.getInstance().getCoralPole(),
                        GameData.getInstance().getDriveMode()));

        // Set reef position
        this.driverController.povUp().onTrue(runOnce(() -> GameData.getInstance().setReefStationIndex(1)));
        this.driverController.povDown().onTrue(runOnce(() -> GameData.getInstance().setReefStationIndex(-1)));
        this.driverController.povLeft().onTrue(runOnce(() -> GameData.getInstance().setCoralPole(CoralPole.LEFT)));
        this.driverController.povRight().onTrue(runOnce(() -> GameData.getInstance().setCoralPole(CoralPole.RIGHT)));
    }

    /**
     * 
     */
    private void configureOperatorBindings() {
        this.operatorController.a()
                .whileTrue(runOnce(() -> this.elevatorSubsystem.addAction(Action.MOVE_TO_IDLE)));
        this.operatorController.x()
                .whileTrue(runOnce(() -> this.elevatorSubsystem.addAction(Action.MOVE_TO_CORAL_1)));
        this.operatorController.b()
                .whileTrue(runOnce(() -> this.elevatorSubsystem.addAction(Action.MOVE_TO_CORAL_2)));
        this.operatorController.y()
                .whileTrue(runOnce(() -> this.elevatorSubsystem.addAction(Action.MOVE_TO_CORAL_3)));
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
}
