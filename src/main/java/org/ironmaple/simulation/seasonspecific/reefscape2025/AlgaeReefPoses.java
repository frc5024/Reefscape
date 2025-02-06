package org.ironmaple.simulation.seasonspecific.reefscape2025;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class AlgaeReefPoses {
    /**
     * 
     */
    public static List<Pose3d> getPoses() {
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
}
