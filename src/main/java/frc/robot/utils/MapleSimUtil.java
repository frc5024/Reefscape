package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.MapleSimConstants;
import frc.robot.modules.elevator.ElevatorVisualizer;

public class MapleSimUtil {
    private static SwerveDriveSimulation swerveDriveSimulation;
    private static IntakeSimulation algaeIntakeSimulation;
    private static IntakeSimulation coralIntakeSimulation;

    /**
     * 
     */
    public static SwerveDriveSimulation getSwerveDriveSimulation() {
        if (swerveDriveSimulation == null) {
            swerveDriveSimulation = new SwerveDriveSimulation(MapleSimConstants.mapleSimConfig,
                    new Pose2d(0, 0, new Rotation2d()));
        }

        return swerveDriveSimulation;
    }

    /**
     * 
     */
    public static IntakeSimulation getAlgaeIntakeSimulation() {
        if (algaeIntakeSimulation == null) {
            algaeIntakeSimulation = IntakeSimulation.OverTheBumperIntake("Algae", getSwerveDriveSimulation(),
                    Meters.of(0.5),
                    Meters.of(0.4), IntakeSimulation.IntakeSide.BACK, 1);
        }

        return algaeIntakeSimulation;
    }

    /**
     * 
     */
    public static IntakeSimulation getCoralIntakeSimulation() {
        if (coralIntakeSimulation == null) {
            coralIntakeSimulation = IntakeSimulation.OverTheBumperIntake("Coral", getSwerveDriveSimulation(),
                    Meters.of(0.5),
                    Meters.of(0.4), IntakeSimulation.IntakeSide.FRONT, 1);
        }

        return coralIntakeSimulation;
    }

    /**
     * 
     */
    public static void ejectAlgae(Pose3d algaePose) {
        ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));

        Transform3d algaeTransform = new Transform3d(
                new Pose3d(getSwerveDriveSimulation().getSimulatedDriveTrainPose()), algaePose);

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(algaeTransform.getX(), algaeTransform.getY()),
                        getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(algaeTransform.getZ()),
                        MetersPerSecond.of(2),
                        Degrees.of(0.0)));
    }

    /**
     * 
     */
    public static void ejectCoral(Pose3d coralPose) {
        Transform3d coralTransform = new Transform3d(
                new Pose3d(getSwerveDriveSimulation().getSimulatedDriveTrainPose()), coralPose);

        double angle = ElevatorVisualizer.getOuttakeAngle("Measured");

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(coralTransform.getX(), coralTransform.getY()),
                        getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(coralTransform.getZ()),
                        MetersPerSecond.of(2),
                        Degrees.of(angle)));
    }
}
