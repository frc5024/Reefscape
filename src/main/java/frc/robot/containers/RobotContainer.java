package frc.robot.containers;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected SwerveDriveSubsystem swerveDriveSubsystem;

    /* Controllers */
    private final CommandXboxController controller = new CommandXboxController(0);

    /* Autonomous */
    AutoBuilder autoBuilder;
    LoggedDashboardChooser<Command> autonomousChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autonomousChooser.get();
    }
}
