package frc.robot.modules.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * 
 */
public interface VisionModuleIO {
    public enum PoseObservationType {
        SINGLE_TAG,
        MULTI_TAG
    }

    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public int bestTargetId = 0;
        public boolean hasTarget = false;
        public Pose3d bestTargetPose = new Pose3d();
    }

    /**
     * Represents the angle to a simple target, not used for pose estimation.
     */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    /**
     * Represents a robot pose sample used for pose estimation.
     */
    public static record PoseObservation(
            double timestamp,
            Pose3d cameraPose,
            double latencySecs,
            double averageAmbiguity,
            double reprojectionError,
            long tagsSeenBitMap,
            int numTags,
            double averageTagDistance,
            PoseObservationType type) {
    }

    /**
     * 
     */
    public static record BestTargetId(int bestTargetId) {
    }

    /**
     * 
     */
    public static record BestTargetPose(Pose3d bestTargetPose) {

    }

    /**
     * 
     */
    abstract public double getHeight();

    abstract public String getName();

    abstract public double getPitch();

    abstract public Transform3d getRobotToCamera();

    abstract public double getYaw();

    /**
     * 
     */
    public default void updateInputs(VisionIOInputs inputs) {
    }
}