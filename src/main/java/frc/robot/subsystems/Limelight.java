package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

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

    @Override
    public void periodic() {
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
        Pose3d teafaadw = LimelightHelpers.getTargetPose3d_RobotSpace("");

        SmartDashboard.putNumber("TargetPose_CameraSpace yaw", botPose[4]);
        SmartDashboard.putNumber("BotPose3d_TargetSpace yaw", Units.radiansToDegrees(botPose3D.getRotation().getZ()));
        SmartDashboard.putNumber("TargetPose3d_RobotSpace yaw", Units.radiansToDegrees(teafaadw.getRotation().getZ()));

        SmartDashboard.putNumber("TargetPose3d_RobotSpace distance from", botPose3D.getZ());

    }
}
