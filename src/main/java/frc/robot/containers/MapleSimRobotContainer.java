package frc.robot.containers;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.modules.algae.AlgaeModuleIOSim;
import frc.robot.modules.coral.CoralModuleIOSim;
import frc.robot.modules.elevator.ElevatorModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIOSim;
import frc.robot.modules.swerve.SwerveModuleIOMapleSim;
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

        this.visionSubsystem = new VisionSubsystem(this.swerveDriveSubsystem,
                swerveDriveSimulation::getSimulatedDriveTrainPose, this.swerveDriveSubsystem::getRotation);

        this.algaeSubsystem = new AlgaeSubsystemSim(new AlgaeModuleIOSim());
        this.coralSubsystem = new CoralSubsystemSim(new CoralModuleIOSim());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorModuleIOSim(), this.algaeSubsystem::hasAlgae,
                this.coralSubsystem::hasCoral);

        registerNamedCommands();
        configureAutoBuilder();
        configureBindings();

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
        NamedCommands.registerCommand("ElevatorL0", new InstantCommand(() -> {
            this.elevatorSubsystem.addAction(ElevatorSubsystem.Action.MOVE_TO_BOTTOM);
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
