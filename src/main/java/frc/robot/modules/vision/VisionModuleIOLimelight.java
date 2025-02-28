package frc.robot.modules.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.camera.Camera;
import frc.robot.utils.LimelightHelpers;

/** IO implementation for real Limelight hardware. */
public class VisionModuleIOLimelight implements VisionModuleIO {
    protected final Camera camera;
    private final Supplier<Rotation2d> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name             The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for
     *                         MegaTag 2.
     */
    public VisionModuleIOLimelight(Camera camera, Supplier<Rotation2d> rotationSupplier) {
        this.camera = camera;
        this.rotationSupplier = rotationSupplier;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(this.camera.getName());
        this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        this.latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        this.txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        this.tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        this.megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        this.megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
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
        // Update connection status based on whether an update has been seen in the last
        // 250ms
        inputs.connected = ((RobotController.getFPGATime() - this.latencySubscriber.getLastChange()) / 1000) < 250;
        inputs.bestTargetId = (int) LimelightHelpers.getFiducialID(getName());
        inputs.bestTargetPose = LimelightHelpers.getTargetPose3d_RobotSpace(getName());
        inputs.processor = Camera.Processor.LIMELIGHT;

        // Update target observation
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(this.txSubscriber.get()),
                Rotation2d.fromDegrees(this.tySubscriber.get()));

        // Update orientation for MegaTag 2
        this.orientationPublisher.accept(
                new double[] { this.rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });
        NetworkTableInstance.getDefault()
                .flush(); // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : this.megatag1Subscriber.readQueue()) {
            if (rawSample.value.length == 0)
                continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(
                    new PoseObservation(
                            // Timestamp, based on server timestamp of publish and latency
                            rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                            // 3D pose estimate
                            parsePose(rawSample.value),

                            // Ambiguity, using only the first tag because ambiguity isn't applicable for
                            // multitag
                            rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

                            // Tag count
                            (int) rawSample.value[7],

                            // Average tag distance
                            rawSample.value[9],

                            // Observation type
                            PoseObservationType.MEGATAG_1));
        }
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0)
                continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(
                    new PoseObservation(
                            // Timestamp, based on server timestamp of publish and latency
                            rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                            // 3D pose estimate
                            parsePose(rawSample.value),

                            // Ambiguity, zeroed because the pose is already disambiguated
                            0.0,

                            // Tag count
                            (int) rawSample.value[7],

                            // Average tag distance
                            rawSample.value[9],

                            // Observation type
                            PoseObservationType.MEGATAG_2));
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

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }
}