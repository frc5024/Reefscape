package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.DriveNearestCoralStationCommand;
import frc.robot.commands.DriveProcessorCommand;
import frc.robot.commands.DriveToBestTagCommand;
import frc.robot.commands.DriveToReefStationCommand;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

    /* Controllers */
    private final CommandXboxController controller = new CommandXboxController(0);

    /* Autonomous */
    AutoBuilder autoBuilder;
    LoggedDashboardChooser<Command> autonomousChooser;

    /* */
    private SendableChooser<Integer> reefStationChooser;
    private SendableChooser<Integer> reefPoleChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //
        this.reefStationChooser = new SendableChooser<>();
        this.reefStationChooser.setDefaultOption("Position 1", 0);
        this.reefStationChooser.addOption("Position 2", 1);
        this.reefStationChooser.addOption("Position 3", 2);
        this.reefStationChooser.addOption("Position 4", 3);
        this.reefStationChooser.addOption("Position 5", 4);
        this.reefStationChooser.addOption("Position 6", 5);
        SmartDashboard.putData(this.reefStationChooser);

        this.reefPoleChooser = new SendableChooser<>();
        this.reefPoleChooser.setDefaultOption("Left", 0);
        this.reefPoleChooser.addOption("Right", 2);
        SmartDashboard.putData(this.reefPoleChooser);
    }

    /**
     * 
     */
    protected void configureAutoBuilder() {
        this.autoBuilder = new AutoBuilder(this.swerveDriveSubsystem);
        this.autoBuilder.configureAutonomous();
        this.autonomousChooser = autoBuilder.getAutonomousChooser();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    protected void configureButtonBindings() {
        // Default command, normal field-relative drive
        swerveDriveSubsystem.setDefaultCommand(
                SwerveDriveCommands.joystickDrive(
                        swerveDriveSubsystem,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        SwerveDriveCommands.joystickDriveAtAngle(
                                swerveDriveSubsystem,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(swerveDriveSubsystem::stopWithX, swerveDriveSubsystem));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> swerveDriveSubsystem.setPose(
                                        new Pose2d(swerveDriveSubsystem.getPose().getTranslation(), new Rotation2d())),
                                swerveDriveSubsystem)
                                .ignoringDisable(true));

        // Drive to nearest coral station
        controller
                .x()
                .whileTrue(new DriveNearestCoralStationCommand(this.swerveDriveSubsystem));

        // Drive to processor station
        controller
                .y()
                .whileTrue(new DriveProcessorCommand(this.swerveDriveSubsystem));

        // Drive to selected reef station
        controller
                .rightTrigger()
                .whileTrue(new DriveToReefStationCommand(this.swerveDriveSubsystem, this.swerveDriveSubsystem::getPose,
                        this.reefStationChooser::getSelected, this.reefPoleChooser::getSelected));

        // Drive to best apriltag
        controller
                .rightBumper()
                .whileTrue(new DriveToBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, VisionConstants.DATA_FROM_CAMERA));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autonomousChooser.get();
    }

    /**
     * Called when the alliance reported by the driverstation/FMS changes -
     * Simulation Only
     * 
     * @param alliance new alliance value
     */
    public void onAllianceChanged(Alliance alliance, int location) {
        location -= 1;

        Pose2d pose2d = AllianceFlipUtil.apply(RobotConstants.TUNING_MODE ? FieldConstants.TUNING_POSES[location]
                : FieldConstants.STATION_POSES[location]);
        this.swerveDriveSubsystem.resetPosition(pose2d);
        resetSimulationField(pose2d);
    }

    /**
     * Maple Sim Routines
     */
    abstract public void displaySimFieldToAdvantageScope();

    abstract public void resetSimulationField(Pose2d pose2d);
}
