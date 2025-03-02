package frc.robot.containers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.commands.vision.DriveFromBestTagCommand;
import frc.robot.controls.ButtonBindings;
import frc.robot.controls.GameData;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

abstract public class RobotContainer {
    /* Subsystems */
    protected Coral coralSubsystem = new Coral();
    protected Elevator elevatorSubsystem = Elevator.getInstance();
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

    /* Autonomous */
    private AutoBuilder autoBuilder;
    private LoggedDashboardChooser<Command> autonomousChooser;

    private final Rumble rumble = Rumble.getInstance();
    private final Limelight limelightSubsystem = new Limelight();
    private final LEDs s_LEDs = LEDs.getInstance();

    boolean visionMode = false;
    String mode;

    /**
     * This constructor should remain empty and be overriden
     */
    public RobotContainer() {
    }

    /**
     *
     */
    protected void configureAutoBuilder() {
        this.autoBuilder = new AutoBuilder(this.swerveDriveSubsystem, this.coralSubsystem, this.elevatorSubsystem);
        this.autoBuilder.configureAutonomous();
        this.autonomousChooser = this.autoBuilder.getAutonomousChooser();
    }

    /**
     * 
     */
    protected void configureBindings() {
        ButtonBindings buttonBindings = new ButtonBindings(this.swerveDriveSubsystem,
                this.coralSubsystem, this.elevatorSubsystem, this.visionSubsystem);

        CommandXboxController commandXboxController = RobotConstants.TUNING_MODE
                ? buttonBindings.getTestController()
                : buttonBindings.getDriverController();

        // Drive suppliers
        DoubleSupplier controllerX = () -> -commandXboxController.getLeftY();
        DoubleSupplier controllerY = () -> -commandXboxController.getLeftX();
        DoubleSupplier controllerOmega = () -> -commandXboxController.getRightX();

        Command closedLoopDrive = SwerveDriveCommands.drive(this.swerveDriveSubsystem, controllerX, controllerY,
                controllerOmega, false);

        // Default command, normal field-relative drive
        this.swerveDriveSubsystem.setDefaultCommand(closedLoopDrive);
    }

    public Command getAutonomousCommand() {
        return this.autonomousChooser.get();
    }

    /**
     * 
     */
    protected void registerNamedCommands() {
        NamedCommands.registerCommand("ScoreCoral", coralSubsystem.outtakeCommand());
        NamedCommands.registerCommand("IntakeCoral", coralSubsystem.intakeCommand());
        NamedCommands.registerCommand("DriveRightTag",
                new DriveFromBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, false, GameData.getInstance().getGamePieceMode()));
        NamedCommands.registerCommand("DriveLefttTag",
                new DriveFromBestTagCommand(this.swerveDriveSubsystem, this.visionSubsystem,
                        this.swerveDriveSubsystem::getPose, true, GameData.getInstance().getGamePieceMode()));
    }

    /**
     * Maple Sim Routines
     */
    abstract public void displaySimFieldToAdvantageScope();

    abstract public void resetSimulationField(Pose2d pose2d);
}
