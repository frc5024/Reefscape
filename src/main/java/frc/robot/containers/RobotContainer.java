package frc.robot.containers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.tuning.TuningCommand;
import frc.robot.commands.tuning.TuningSwerveCommand;
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

    /* Alerts */
    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
    private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
            AlertType.kWarning);

    /* Autonomous */
    AutoBuilder autoBuilder;
    LoggedDashboardChooser<Command> autonomousChooser;

    /* Controllers */
    CommandXboxController driverController;
    CommandXboxController operatorController;

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

        this.driverController = commandXboxController;
        this.operatorController = buttonBindings.getOperatorController();

        // Drive suppliers
        DoubleSupplier controllerX = () -> -commandXboxController.getLeftY();
        DoubleSupplier controllerY = () -> -commandXboxController.getLeftX();
        DoubleSupplier controllerOmega = () -> -commandXboxController.getRightX();

        Command closedLoopDrive = SwerveDriveCommands.drive(this.swerveDriveSubsystem, controllerX, controllerY,
                controllerOmega, false);

        Command openLoopDrive = SwerveDriveCommands.drive(this.swerveDriveSubsystem, controllerX, controllerY,
                controllerOmega, true);

        Command tuningCommand = new TuningCommand(this.swerveDriveSubsystem, algaeSubsystem, coralSubsystem,
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
     * 
     */
    public void updateAlerts() {
        // Controller disconnected alerts
        this.driverDisconnected.set(!DriverStation.isJoystickConnected(this.driverController.getHID().getPort()));
        this.operatorDisconnected.set(!DriverStation.isJoystickConnected(this.operatorController.getHID().getPort()));
    }

    /**
     * Maple Sim Routines
     */
    abstract public void displaySimFieldToAdvantageScope();

    abstract public void resetSimulationField(Pose2d pose2d);
}
