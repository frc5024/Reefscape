package frc.robot.containers;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveDriveCommands;
import frc.robot.commands.tuning.TuningCommand;
import frc.robot.controls.ButtonBindingsSim;
import frc.robot.modules.algae.AlgaeModuleIOSim;
import frc.robot.modules.climb.ClimbModuleIOSim;
import frc.robot.modules.coral.CoralModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIOSim;
import frc.robot.modules.swerve.SwerveModuleIOMapleSim;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.simulation.AlgaeSubsystemSim;
import frc.robot.subsystems.simulation.CoralSubsystemSim;
import frc.robot.utils.MapleSimUtil;

/**
 * 
 */
public class MapleSimRobotContainer extends RobotContainer {
    /**
     * 
     */
    public MapleSimRobotContainer() {
        super();

        // create a maple-sim swerve drive simulation instance
        SwerveDriveSimulation swerveDriveSimulation = MapleSimUtil.getSwerveDriveSimulation();
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIOSim(swerveDriveSimulation.getGyroSimulation()),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[0]),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[1]),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[2]),
                new SwerveModuleIOMapleSim(swerveDriveSimulation.getModules()[3]));

        this.visionSubsystem = new VisionSubsystem(VisionConstants.SIMULATION_CAMERAS, this.swerveDriveSubsystem,
                swerveDriveSimulation::getSimulatedDriveTrainPose, this.swerveDriveSubsystem::getRotation);

        this.algaeSubsystem = new AlgaeSubsystemSim(new AlgaeModuleIOSim());
        this.climbSubsystem = new ClimbSubsystem(new ClimbModuleIOSim());
        this.coralSubsystem = new CoralSubsystemSim(new CoralModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim(), this.algaeSubsystem::hasAlgae,
                this.coralSubsystem::hasCoral);

        // If simulation set coral in
        this.coralSubsystem.setHasCoral(true);
        this.algaeSubsystem.setHasAlgae(false);

        registerNamedCommands();
        configureAutoBuilder();
        configureBindings();

        // Initiate the LEDSubsystem
        LEDSubsystem.getInstance();
    }

    @Override
    public void configureBindings() {
        ButtonBindingsSim buttonBindings = new ButtonBindingsSim(this.swerveDriveSubsystem, this.algaeSubsystem,
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

        Command tuningCommand = new TuningCommand(this.swerveDriveSubsystem, this.elevatorSubsystem, controllerX,
                controllerY, controllerOmega, commandXboxController);

        // Default command, normal field-relative drive
        this.swerveDriveSubsystem.setDefaultCommand(RobotConstants.TUNING_MODE ? tuningCommand : closedLoopDrive);
    }

    /**
     * 
     */
    public void displaySimFieldToAdvantageScope() {
        if (RobotConstants.currentMode != RobotConstants.Mode.SIM)
            return;

        Logger.recordOutput("FieldSimulation/RobotPosition",
                MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    @Override
    public void registerNamedCommands() {
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
     * 
     */
    public void resetSimulationField(Pose2d pose2d) {
        if (RobotConstants.currentMode != RobotConstants.Mode.SIM)
            return;

        MapleSimUtil.getSwerveDriveSimulation().setSimulationWorldPose(pose2d);
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}
