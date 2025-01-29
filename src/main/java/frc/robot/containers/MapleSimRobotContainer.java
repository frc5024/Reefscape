package frc.robot.containers;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.RobotConstants;
import frc.robot.modules.algae.AlgaeIntakeModuleIOSim;
import frc.robot.modules.gyro.GyroModuleIOSim;
import frc.robot.modules.swerve.SwerveModuleIOMapleSim;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.simulation.AlgaeIntakeSubsystemSim;
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

        configureAutoBuilder();
        configureButtonBindings();
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
