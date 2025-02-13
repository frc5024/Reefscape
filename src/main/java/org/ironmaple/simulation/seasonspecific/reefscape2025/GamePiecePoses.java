package org.ironmaple.simulation.seasonspecific.reefscape2025;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * Poses of coral and algae at game startup - used for simulation blue alliance
 */
public class GamePiecePoses {
    /**
     *
     */
    public static List<Pose3d> getAlgaeReefPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();

        poses.add(new Pose3d(Units.inchesToMeters(152.00), Units.inchesToMeters(158.50), Units.inchesToMeters(52),
                new Rotation3d()));
        poses.add(new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Units.inchesToMeters(36),
                new Rotation3d()));
        poses.add(new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Units.inchesToMeters(52),
                new Rotation3d()));
        poses.add(new Pose3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Units.inchesToMeters(36),
                new Rotation3d()));
        poses.add(new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Units.inchesToMeters(52),
                new Rotation3d()));
        poses.add(new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Units.inchesToMeters(36),
                new Rotation3d()));

        return poses;
    }

    /**
     *
     */
    public static List<Pose3d> getCoralStationPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();

        poses.add(new Pose3d(Units.inchesToMeters(48.0), Units.inchesToMeters(6.0), Units.inchesToMeters(46.0),
                new Rotation3d(0.0, Units.degreesToRadians(-35.0), Units.degreesToRadians(55.0))));
        poses.add(new Pose3d(Units.inchesToMeters(52.0), Units.inchesToMeters(304.0), Units.inchesToMeters(43.0),
                new Rotation3d(0.0, Units.degreesToRadians(-35.0), Units.degreesToRadians(-55.0))));

        return poses;
    }
}
