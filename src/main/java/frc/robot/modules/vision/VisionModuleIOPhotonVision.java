package frc.robot.modules.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
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
    private final List<VisionModuleIO.PoseObservation> poseObservations = new ArrayList<>();

    /**
     * 
     */
    public VisionModuleIOPhotonVision(Camera camera) {
        this.camera = camera;
        this.photonCamera = new PhotonCamera(camera.getName());
        this.photonCamera.setPipelineIndex(camera.getPipelineIndex());

        if (this.camera.getType() == Type.APRILTAG) {
            // Don't pass the robot to camera transform as we will work with the estimated
            // camera poses and later transform them to the robot's frame
            this.photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_FIELD_LAYOUT,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());
            this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        } else {
            this.photonPoseEstimator = null;
        }

        // flush any old results from previous results
        this.photonCamera.getAllUnreadResults();
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
    public Transform3d getRobotToCamera() {
        return this.camera.getRobotToCamera();
    }

    @Override
    public double getYaw() {
        return this.camera.getYaw();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        this.poseObservations.clear();

        inputs.connected = this.photonCamera.isConnected();

        // Read new camera observations
        for (PhotonPipelineResult result : this.photonCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEstimate = this.photonPoseEstimator.update(result);

            if (result.hasTargets()) {
                inputs.bestTargetId = result.getBestTarget().getFiducialId();
                inputs.hasTarget = true;
                inputs.bestTargetPose = VisionConstants.TAG_FIELD_LAYOUT
                        .getTagPose(result.getBestTarget().getFiducialId()).get();
            } else {
                inputs.bestTargetId = -1;
                inputs.hasTarget = false;
                inputs.bestTargetPose = null;
            }

            visionEstimate.ifPresent(estimate -> {
                long tagsSeenBitMap = 0;
                double averageAmbiguity = 0.0;
                double averageTagDistance = 0.0;

                for (int i = 0; i < estimate.targetsUsed.size(); i++) {
                    tagsSeenBitMap |= 1L << estimate.targetsUsed.get(i).getFiducialId();
                    averageAmbiguity += estimate.targetsUsed.get(i).getPoseAmbiguity();
                    averageTagDistance += estimate.targetsUsed.get(i).getBestCameraToTarget().getTranslation()
                            .getNorm();
                }
                averageAmbiguity /= estimate.targetsUsed.size();
                averageTagDistance /= estimate.targetsUsed.size();

                this.poseObservations.add(
                        new PoseObservation(
                                result.getTimestampSeconds(),
                                estimate.estimatedPose,
                                Timer.getFPGATimestamp() - result.getTimestampSeconds(),
                                averageAmbiguity,
                                result.multitagResult.isPresent()
                                        ? result.multitagResult.get().estimatedPose.bestReprojErr
                                        : 0.0,
                                tagsSeenBitMap,
                                estimate.targetsUsed.size(),
                                averageTagDistance,
                                estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                                        ? PoseObservationType.MULTI_TAG
                                        : PoseObservationType.SINGLE_TAG));
            });
        }

        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    }
}
