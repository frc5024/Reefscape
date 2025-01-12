package frc.robot.containers;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.modules.gyro.GyroModuleIOSim;
import frc.robot.modules.swerve.SwerveModuleIOMapleSim;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class SimulatedRobotContainer extends RobotContainer {
    private SwerveDriveSimulation driveSimulation = null;

    /**
     * 
     */
    public SimulatedRobotContainer() {
        super();

        // create a maple-sim swerve drive simulation instance
        this.driveSimulation = new SwerveDriveSimulation(Constants.mapleSimConfig,
                new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        this.swerveDriveSubsystem = new SwerveDriveSubsystem(
                new GyroModuleIOSim(driveSimulation.getGyroSimulation()),
                new SwerveModuleIOMapleSim(driveSimulation.getModules()[0]),
                new SwerveModuleIOMapleSim(driveSimulation.getModules()[1]),
                new SwerveModuleIOMapleSim(driveSimulation.getModules()[2]),
                new SwerveModuleIOMapleSim(driveSimulation.getModules()[3]));

        configureAutoBuilder();
        configureButtonBindings();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}
