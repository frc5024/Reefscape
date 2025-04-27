package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

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
    public boolean done = false;

    double distanceDiff;
    double strafeDiff;
    double rotationDiff;
    boolean isVisionActivated = false;

    private static Limelight mInstance = null;

    boolean isTagSeen = false;

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    private Limelight() {
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
        return limelightTable.getEntry("tid").getDouble(-1);
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

    public boolean inPosition() {
        return getXPos() && getZPos() && getRotationPos();
    }

    public void pathIsDone(boolean done) {
        this.done = done;
    }

    public boolean getPathIsDone() {
        return done;
    }

    public void setDistanceDiff(double diff) {
        distanceDiff = diff;
    }

    public void setStrafeDiff(double diff) {
        strafeDiff = diff;
    }

    public void setRotationDiff(double diff) {
        rotationDiff = diff;
    }

    public double getDistanceDiff() {
        return distanceDiff;
    }

    public double getStrafeDiff() {
        return strafeDiff;
    }

    public double getRotationDiff() {
        return rotationDiff;
    }

    public void isVisionActivated(boolean isVisionActivated) {
        this.isVisionActivated = isVisionActivated;
    }

    public boolean getIsVisionActive() {
        return isVisionActivated;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("rotationPos", rotatePos);
        SmartDashboard.putBoolean("xPos (Left Right)", xPos);
        SmartDashboard.putBoolean("zPos (distance)", zPos);

        SmartDashboard.putBoolean("is Tag Visable", isTargetVisible());
        SmartDashboard.putNumber("Visable Tag", getAprilTagID());

        /*Logger.recordOutput("Subsystems/Limelight/BestTargetId", LimelightHelpers.getFiducialID("limelight"));
        Logger.recordOutput("Subsystems/Limelight/CurrentPose", LimelightHelpers.getBotPose3d("limelight"));

        Logger.recordOutput("Subsystems/Auto/Distance Difference (0.03 tol)", getDistanceDiff());
        Logger.recordOutput("Subsystems/Auto/Strafe Difference (0.02 tol)", getStrafeDiff());
        Logger.recordOutput("Subsystems/Auto/Rotation Difference (1.6 tol)", getRotationDiff());

        Logger.recordOutput("Subsystems/Auto/at Distance", getZPos());
        Logger.recordOutput("Subsystems/Auto/at Strafe", getXPos());
        Logger.recordOutput("Subsystems/Auto/at Rotation", getRotationPos());
        Logger.recordOutput("Subsystems/Auto/Is In Position?", inPosition());

        Logger.recordOutput("Subsystems/Auto/Is Vision Running?", getIsVisionActive()); */
    }
}
