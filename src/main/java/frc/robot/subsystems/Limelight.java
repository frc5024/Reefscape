package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable limelightTable;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean isTargetVisible() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    public double getX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    public double getAprilTagID() {
        return limelightTable.getEntry("tid").getDouble(-1.0); // -1 if no target
    }

    // public double[] botPose() {
    // return limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new
    // double[6]);
    // }
}
