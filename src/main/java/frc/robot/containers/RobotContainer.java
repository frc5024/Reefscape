package frc.robot.containers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TuningCommand;
import frc.robot.commands.TuningSwerveCommand;
import frc.robot.controls.ButtonBindings;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
abstract public class RobotContainer {
    /* Subsystems */
    protected AlgaeSubsystem algaeSubsystem;
    protected CoralSubsystem coralSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

    /* Autonomous */
    AutoBuilder autoBuilder;
    LoggedDashboardChooser<Command> autonomousChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
    }

    /**
     * ; *
     */
    protected void configureAutoBuilder() {
        this.autoBuilder = new AutoBuilder(this.swerveDriveSubsystem, this.algaeSubsystem,
                this.coralSubsystem, this.elevatorSubsystem);
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
        ButtonBindings buttonBindings = new ButtonBindings(this.swerveDriveSubsystem,
                this.algaeSubsystem,
                this.coralSubsystem, this.elevatorSubsystem, this.visionSubsystem);

        CommandXboxController commandXboxController = RobotConstants.TUNING_MODE
                ? buttonBindings.getTestController()
                : buttonBindings.getDriverController();

        // Drive suppliers
        DoubleSupplier controllerX = () -> -commandXboxController.getLeftY();
        DoubleSupplier controllerY = () -> -commandXboxController.getLeftX();
        DoubleSupplier controllerOmega = () -> -commandXboxController.getRightX();
        Supplier<Rotation2d> rotationSupplier = () -> this.swerveDriveSubsystem.getPose().getRotation();

        Command closedLoopDrive = SwerveDriveCommands.closedLoopDrive(swerveDriveSubsystem, controllerX, controllerY,
                controllerOmega);

        Command openLoopDrive = new TeleopDriveCommand(this.swerveDriveSubsystem, rotationSupplier, controllerX,
                controllerY, controllerOmega);

        Command tuningCommand = new TuningCommand(swerveDriveSubsystem, algaeSubsystem, coralSubsystem,
                elevatorSubsystem, controllerX, controllerY, controllerOmega, commandXboxController);

        Command tuningSwerveCommand = new TuningSwerveCommand(swerveDriveSubsystem, controllerX, controllerY,
                controllerOmega, commandXboxController);

        // Default command, normal field-relative drive
        swerveDriveSubsystem.setDefaultCommand(RobotConstants.TUNING_MODE ? tuningCommand : closedLoopDrive);
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
        int index = alliance == Alliance.Blue ? 0 : 1;
        location -= 1;

        Pose2d pose2d = RobotConstants.TUNING_MODE ? FieldConstants.TUNING_POSES[index][location]
                : FieldConstants.STATION_POSES[index][location];
        this.swerveDriveSubsystem.resetPosition(pose2d);
        resetSimulationField(pose2d);
    }

    abstract public void registerNamedCommands();

    /**
     * Maple Sim Routines
     */
    abstract public void displaySimFieldToAdvantageScope();

    abstract public void resetSimulationField(Pose2d pose2d);
}
