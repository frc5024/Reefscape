package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private final NetworkTable limelightTable;

    boolean rotatePos = false;
    boolean xPos = false;
    boolean zPos = false;

    boolean isTagSeen = false;

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
        return limelightTable.getEntry("tid").getDouble(-100); // -100 if no target
    }

    public void setRotationPos(boolean rotationPos) {
        rotatePos = rotationPos;
    }

    public boolean getRotationPos() {
        return rotatePos;
    }

    public void setXPos(boolean xPos) {
        this.xPos = xPos;
    }

    public boolean getXPos() {
        return xPos;
    }

    public void setZPos(boolean zPos) {
        this.zPos = zPos;
    }

    public boolean getZPos() {
        return zPos;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("rotationPos", rotatePos);
        SmartDashboard.putBoolean("xPos (Left Right)", xPos);
        SmartDashboard.putBoolean("zPos (distance)", zPos);

        SmartDashboard.putBoolean("is Tag Visable", isTargetVisible());
        SmartDashboard.putNumber("Visable Tag", getAprilTagID());
        Logger.recordOutput("Subsystems/Limelight/BestTargetId", LimelightHelpers.getFiducialID("limelight"));
        Logger.recordOutput("Subsystems/Limelight/CurrentPose", LimelightHelpers.getBotPose3d("limelight"));
    }
}
