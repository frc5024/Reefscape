package frc.robot.containers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.controls.ButtonBindings;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

abstract public class RobotContainer {
    /* Subsystems */
    protected Coral coral;
    protected Elevator elevator;
    protected Rumble rumble;
    protected Limelight limelightSubsystem;
    protected LEDs lEDs;

    protected AlgaeSubsystem algaeSubsystem;
    protected CoralSubsystem coralSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected SwerveDriveSubsystem swerveDriveSubsystem;
    protected VisionSubsystem visionSubsystem;

    /* Autonomous */
    private AutoBuilder autoBuilder;
    private LoggedDashboardChooser<Command> autonomousChooser;

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
        this.autoBuilder = new AutoBuilder(this.swerveDriveSubsystem, this.coral, this.elevator);
        this.autoBuilder.configureAutonomous();
        this.autonomousChooser = this.autoBuilder.getAutonomousChooser();
    }

    /**
     * 
     */
    protected void configureBindings() {
        ButtonBindings buttonBindings = new ButtonBindings(this.swerveDriveSubsystem,
                this.coral, this.elevator, this.limelightSubsystem);

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

    abstract public void registerNamedCommands();

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

    /**
     * Maple Sim Routines
     */
    abstract public void displaySimFieldToAdvantageScope();

    abstract public void resetSimulationField(Pose2d pose2d);
}
