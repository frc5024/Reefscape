package frc.robot.modules.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.camera.Camera;
import frc.lib.camera.Camera.Type;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModuleIOPhotonVision implements VisionModuleIO {
    protected final Camera camera;
    protected final PhotonCamera photonCamera;

    protected final PhotonPoseEstimator photonPoseEstimator;
    protected final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose;

    /**
     * 
     */
    public VisionModuleIOPhotonVision(Camera camera) {
        this.camera = camera;
        this.photonCamera = new PhotonCamera(camera.getName());
        this.photonCamera.setPipelineIndex(camera.getPipelineIndex());

        if (this.camera.getType() == Type.APRILTAG) {
            this.photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_FIELD_LAYOUT,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera.getRobotToCamera());
            this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        } else {
            this.photonPoseEstimator = null;
        }

        this.atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    }

    @Override
    public PhotonTrackedTarget getBestTarget() {
        return null;
    }

    @Override
    public double getHeight() {
        return this.camera.getHeight();
    }

    @Override
    public String getName() {
        return this.camera.getName();
    }

    @Override
    public double getPitch() {
        return this.camera.getPitch();
    }

    @Override
    public double getYaw() {
        return this.camera.getYaw();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = this.photonCamera.isConnected();

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (PhotonPipelineResult result : this.photonCamera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.bestTargetId = result.getBestTarget().getFiducialId();
                inputs.bestTargetPose = VisionConstants.TAG_FIELD_LAYOUT
                        .getTagPose(result.getBestTarget().getFiducialId()).get();

                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(this.camera.getCameraToRobot());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        robotPose, // 3D pose estimate
                        multitagResult.estimatedPose.ambiguity, // Ambiguity
                        multitagResult.fiducialIDsUsed.size(), // Tag count
                        totalTagDistance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION)); // Observation type
            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = VisionConstants.TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                            tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(this.camera.getCameraToRobot());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    target.poseAmbiguity, // Ambiguity
                                    1, // Tag count
                                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
                                    PoseObservationType.PHOTONVISION)); // Observation type
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}
