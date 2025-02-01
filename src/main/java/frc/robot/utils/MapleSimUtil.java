package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.MapleSimConstants;

public class MapleSimUtil {
    private static SwerveDriveSimulation swerveDriveSimulation;
    private static IntakeSimulation intakeSimulation;

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
    public static IntakeSimulation getIntakeSimulation() {
        if (intakeSimulation == null) {
            intakeSimulation = IntakeSimulation.OverTheBumperIntake("Algae", getSwerveDriveSimulation(), Meters.of(0.5),
                    Meters.of(0.4), IntakeSimulation.IntakeSide.BACK, 1);
        }

        return intakeSimulation;
    }

    /**
     * 
     */
    public static void ejectAlgae() {
        ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(),
                        getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getRotation()
                                .minus(new Rotation2d(180)),
                        Meters.of(0.4), // initial height of the ball, in meters
                        MetersPerSecond.of(3), // initial velocity, in m/s
                        Angle.ofRelativeUnits(0, Units.Degrees)) // shooter angle
                        .withProjectileTrajectoryDisplayCallBack(
                                (poses) -> Logger.recordOutput("successfulShotsTrajectory",
                                        poses.toArray(Pose3d[]::new)),
                                (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
    }

    /**
     * 
     */
    public static void ejectCoral() {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(),
                        getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        getSwerveDriveSimulation().getSimulatedDriveTrainPose().getRotation()
                                .minus(new Rotation2d(180)),
                        Meters.of(0.4), // initial height of the ball, in meters
                        MetersPerSecond.of(3), // initial velocity, in m/s
                        Angle.ofRelativeUnits(0, Units.Degrees)) // shooter angle
                        .withProjectileTrajectoryDisplayCallBack(
                                (poses) -> Logger.recordOutput("successfulShotsTrajectory",
                                        poses.toArray(Pose3d[]::new)),
                                (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
    }
}
