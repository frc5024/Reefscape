package frc.robot.containers;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.modules.algae.AlgaeIntakeModuleIOSim;
import frc.robot.modules.coral.CoralIntakeModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIOSim;
import frc.robot.modules.swerve.SwerveModuleIOMapleSim;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.simulation.AlgaeIntakeSubsystemSim;
import frc.robot.subsystems.simulation.CoralIntakeSubsystemSim;
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

        this.visionSubsystem = new VisionSubsystem(this.swerveDriveSubsystem,
                swerveDriveSimulation::getSimulatedDriveTrainPose, this.swerveDriveSubsystem::getRotation);

        this.algaeIntakeSubsystem = new AlgaeIntakeSubsystemSim(new AlgaeIntakeModuleIOSim());
        this.coralIntakeSubsystem = new CoralIntakeSubsystemSim(new CoralIntakeModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim());

        registerNamedCommands();
        configureAutoBuilder();
        configureButtonBindings();

        // Initiate the LEDSubsystem
        LEDSubsystem.getInstance();

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
        /* Algae Subsystem Commands */
        NamedCommands.registerCommand("Intake Algae",
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }));
        NamedCommands.registerCommand("Eject Algae",
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                }));

        /* Coral Subsystem Commands */
        NamedCommands.registerCommand("Intake Coral",
                new InstantCommand(() -> {
                    this.coralIntakeSubsystem.addAction(CoralIntakeSubsystem.Action.INTAKE);
                }));
        NamedCommands.registerCommand("Eject Coral",
                new InstantCommand(() -> {
                    this.coralIntakeSubsystem.addAction(CoralIntakeSubsystem.Action.EJECT);
                }));

        /* Elevator Subsystem Commands */
        NamedCommands.registerCommand("Move Elevator to Idle",
                new InstantCommand(() -> {
                    this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_IDLE);
                }));
        NamedCommands.registerCommand("Move Elevator to Coral 1",
                new InstantCommand(() -> {
                    this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_1);
                }));
        NamedCommands.registerCommand("Move Elevator to Coral 2",
                new InstantCommand(() -> {
                    this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_2);
                }));
        NamedCommands.registerCommand("Move Elevator to Coral 3",
                new InstantCommand(() -> {
                    this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_CORAL_3);
                }));

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
