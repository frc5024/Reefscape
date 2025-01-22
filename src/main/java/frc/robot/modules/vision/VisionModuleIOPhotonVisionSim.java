package frc.robot.modules.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.camera.Camera;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
public class VisionModuleIOPhotonVisionSim extends VisionModuleIOPhotonVision {
    private final PhotonCameraSim photonCameraSim;
    private final VisionSystemSim visionSystemSim;
    private final Supplier<Pose2d> poseSupplier;

    /**
     * 
     */
    public VisionModuleIOPhotonVisionSim(Camera camera, Supplier<Pose2d> poseSupplier) {
        super(camera);

        this.poseSupplier = poseSupplier;

        SimCameraProperties simCameraProperties = new SimCameraProperties();
        simCameraProperties.setCalibration(VisionConstants.IMG_WIDTH, VisionConstants.IMG_HEIGHT,
                Rotation2d.fromDegrees(VisionConstants.DIAGONAL_FOV));
        simCameraProperties.setCalibError(0.35, 0.10);
        simCameraProperties.setFPS(15);
        simCameraProperties.setAvgLatencyMs(50);
        simCameraProperties.setLatencyStdDevMs(15);

        this.photonCameraSim = new PhotonCameraSim(this.photonCamera, simCameraProperties);

        this.visionSystemSim = new VisionSystemSim(this.camera.getName());
        this.visionSystemSim.addCamera(this.photonCameraSim, this.camera.getRobotToCamera());

        if (this.camera.getType() == Camera.Type.APRILTAG) {
            this.visionSystemSim.addAprilTags(VisionConstants.TAG_FIELD_LAYOUT);

        } else if (this.camera.getType() == Camera.Type.COLOURED_SHAPE) {
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        this.visionSystemSim.update(this.poseSupplier.get());
        super.updateInputs(inputs);
    }
}