package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.camera.Camera;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.modules.vision.VisionIOInputsAutoLogged;
import frc.robot.modules.vision.VisionModuleIO;
import frc.robot.modules.vision.VisionModuleIO.PoseObservation;
import frc.robot.modules.vision.VisionModuleIO.PoseObservationType;
import frc.robot.modules.vision.VisionModuleIOLimelight;
import frc.robot.modules.vision.VisionModuleIOPhotonVision;
import frc.robot.modules.vision.VisionModuleIOPhotonVisionSim;
import frc.robot.utils.LoggedTracer;

public class VisionSubsystem extends SubsystemBase {
    private final String NAME = "Vision";

    /* Alerts */
    private final Alert disconnected = new Alert(NAME + " camera disconnected!", Alert.AlertType.kWarning);

    /* Modules */
    private VisionModuleIO[] visionModules;

    /* Variables */
    private final VisionConsumer visionConsumer;
    private final Supplier<Pose2d> poseSupplier;
    private final VisionIOInputsAutoLogged[] inputs;
    private final double[] lastTimestamps;
    private final Alert[] disconnectedAlerts;

    private final Pose3d robotPoseForCalibration;

    private Pose3d mostRecentBestPose = new Pose3d();
    private double mostRecentBestPoseTimestamp = 0.0;
    private double mostRecentBestPoseStdDev = 0.0;

    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
    private final Map<Pose3d, Double> lastPoseEstimationAcceptedTimes = new HashMap<>();
    private final Map<Pose3d, Double> lastPoseEstimationRejectedTimes = new HashMap<>();

    private final List<Pose3d> allTagPoses = new LinkedList<>();
    private final List<Pose3d> allRobotPoses = new LinkedList<>();
    private final List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    private final List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    private final List<List<Pose3d>> cameraPoses;
    private final List<List<Pose3d>> robotPoses;
    private final List<List<Pose3d>> robotPosesAccepted;
    private final List<List<Pose3d>> robotPosesRejected;
    private final List<List<Pose3d>> tagPoses;

    /**
     * 
     */
    public VisionSubsystem(List<Camera> cameras, VisionConsumer visionConsumer, Supplier<Pose2d> poseSupplier,
            Supplier<Rotation2d> rotationSupplier) {
        this.visionConsumer = visionConsumer;
        this.poseSupplier = poseSupplier;

        this.visionModules = new VisionModuleIO[cameras.size()];
        this.cameraPoses = new ArrayList<List<Pose3d>>(cameras.size());
        this.robotPoses = new ArrayList<List<Pose3d>>(cameras.size());
        this.robotPosesAccepted = new ArrayList<List<Pose3d>>(cameras.size());
        this.robotPosesRejected = new ArrayList<List<Pose3d>>(cameras.size());
        this.tagPoses = new ArrayList<List<Pose3d>>(cameras.size());

        // Setup the vision modules
        for (int cameraIndex = 0; cameraIndex < cameras.size(); cameraIndex++) {
            Camera camera = cameras.get(cameraIndex);
            if (Robot.isReal()) {
                if (camera.getProcessor() == Camera.Processor.LIMELIGHT) {
                    this.visionModules[cameraIndex] = new VisionModuleIOLimelight(camera, rotationSupplier);
                } else if (camera.getProcessor() == Camera.Processor.PHOTONVISION) {
                    this.visionModules[cameraIndex] = new VisionModuleIOPhotonVision(camera);
                }
            } else {
                if (camera.getProcessor() == Camera.Processor.LIMELIGHT) {
                    this.visionModules[cameraIndex] = new VisionModuleIOLimelight(camera, rotationSupplier);
                } else if (camera.getProcessor() == Camera.Processor.PHOTONVISION) {
                    this.visionModules[cameraIndex] = new VisionModuleIOPhotonVisionSim(camera, this.poseSupplier);
                }
            }
        }

        this.lastTimestamps = new double[cameras.size()];

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[this.visionModules.length];
        this.disconnectedAlerts = new Alert[this.visionModules.length];
        for (int i = 0; i < this.visionModules.length; i++) {
            this.inputs[i] = new VisionIOInputsAutoLogged();
            this.disconnectedAlerts[i] = new Alert("Vision camera " + i + " is disconnected.", AlertType.kError);

            this.cameraPoses.add(new ArrayList<>());
            this.robotPoses.add(new ArrayList<>());
            this.robotPosesAccepted.add(new ArrayList<>());
            this.robotPosesRejected.add(new ArrayList<>());
            this.tagPoses.add(new ArrayList<>());
        }

        // robot to camera transformation calibration
        this.robotPoseForCalibration = new Pose3d(
                new Pose2d(
                        Units.inchesToMeters(144.003),
                        Units.inchesToMeters(158.500),
                        Rotation2d.fromDegrees(180))
                        .transformBy(
                                new Transform2d(
                                        RobotConstants.LENGTH_METERS / 2.0,
                                        0,
                                        Rotation2d.fromDegrees(180))));
    }

    @Override
    public void periodic() {
        for (int cameraIndex = 0; cameraIndex < this.visionModules.length; cameraIndex++) {
            VisionIOInputsAutoLogged inputs = this.inputs[cameraIndex];

            this.disconnected.set(!inputs.connected);

            this.visionModules[cameraIndex].updateInputs(inputs);
            Logger.processInputs("Vision/Camera " + this.visionModules[cameraIndex].getName(), inputs);
        }

        // Initialize logging values
        this.allRobotPoses.clear();
        this.allRobotPosesAccepted.clear();
        this.allRobotPosesRejected.clear();
        this.allTagPoses.clear();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < this.visionModules.length; cameraIndex++) {
            String cameraName = this.visionModules[cameraIndex].getName();

            // Update disconnected alert
            Alert disconnectedAlert = this.disconnectedAlerts[cameraIndex];
            disconnectedAlert.set(!inputs[cameraIndex].connected);

            // Initialize logging values
            this.tagPoses.get(cameraIndex).clear();
            this.robotPoses.get(cameraIndex).clear();
            this.robotPosesAccepted.get(cameraIndex).clear();
            this.robotPosesRejected.get(cameraIndex).clear();

            // Loop over pose observations
            for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
                // only process the vision data if the timestamp is newer than the last one
                if (this.lastTimestamps[cameraIndex] < observation.timestamp()) {

                    if (VisionConstants.CALIBRATE_CAMERA_TRANSFORMS) {
                        logCameraTransforms(cameraName, observation);
                    }

                    // Initialize logging values
                    this.lastTimestamps[cameraIndex] = observation.timestamp();
                    cameraPoses.get(cameraIndex).add(observation.cameraPose());
                    Pose3d estimatedRobotPose3d = observation
                            .cameraPose()
                            .plus(this.visionModules[cameraIndex].getRobotToCamera().inverse());
                    Pose2d estimatedRobotPose2d = estimatedRobotPose3d.toPose2d();
                    robotPoses.get(cameraIndex).add(estimatedRobotPose3d);

                    // only update the pose estimator if the vision subsystem is enabled and the
                    // vision's estimated pose is on (or very close to) the field
                    // for multi-tag strategies, ensure the reprojection error is less than the
                    // threshold; for single-tag, ensure the ambiguity is less than the threshold.
                    boolean acceptPose = (observation.type() == PoseObservationType.MULTI_TAG
                            || observation.averageAmbiguity() < VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)
                            && (observation.type() == PoseObservationType.SINGLE_TAG
                                    || Math.abs(observation
                                            .reprojectionError()) < VisionConstants.APRITAG_REPROJECTION_ERROR_THRESHOLD)
                            && poseIsOnField(estimatedRobotPose3d)
                            && arePoseRotationsReasonable(estimatedRobotPose3d);

                    if (acceptPose) {
                        // get tag poses and update last detection times
                        final int finalCameraIndex = cameraIndex;
                        for (int tagID = 1; tagID < 30; tagID++) {
                            if ((observation.tagsSeenBitMap() & (1L << tagID)) != 0) {
                                if (VisionConstants.ENABLE_DETAILED_LOGGING) {
                                    this.lastTagDetectionTimes.put(tagID, Timer.getTimestamp());
                                }
                                Optional<Pose3d> tagPose = VisionConstants.TAG_FIELD_LAYOUT.getTagPose(tagID);
                                tagPose.ifPresent(
                                        (e) -> {
                                            tagPoses.get(finalCameraIndex).add(e);
                                        });
                            }
                        }
                        robotPosesAccepted.get(cameraIndex).add(estimatedRobotPose3d);
                        if (VisionConstants.ENABLE_DETAILED_LOGGING) {
                            this.lastPoseEstimationAcceptedTimes.put(estimatedRobotPose3d, Timer.getTimestamp());
                        }

                        Matrix<N3, N1> stdDev = getStandardDeviations(cameraIndex, observation);
                        this.visionConsumer.accept(
                                estimatedRobotPose2d,
                                observation.timestamp(),
                                stdDev);

                        // if the most-recent "best pose" is too old, capture a new one regardless of
                        // its standard deviation values; otherwise, only capture a new one if its
                        // standard deviation is lower than the current best pose
                        if (this.mostRecentBestPoseTimestamp < Timer.getTimestamp()
                                - VisionConstants.BEST_POSE_TIME_THRESHOLD_SECS
                                || this.mostRecentBestPoseStdDev > stdDev.get(0, 0)) {
                            this.mostRecentBestPose = estimatedRobotPose3d;
                            this.mostRecentBestPoseTimestamp = observation.timestamp();
                            this.mostRecentBestPoseStdDev = stdDev.get(0, 0);
                        }
                    } else {
                        this.robotPosesRejected.get(cameraIndex).add(estimatedRobotPose3d);
                        if (VisionConstants.ENABLE_DETAILED_LOGGING) {
                            this.lastPoseEstimationRejectedTimes.put(estimatedRobotPose3d, Timer.getTimestamp());
                        }
                    }
                }
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + cameraName + "/TagPoses",
                    this.tagPoses.get(cameraIndex).toArray(Pose3d[]::new));
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + cameraName + "/RobotPoses",
                    this.robotPoses.get(cameraIndex)
                            .toArray(new Pose3d[this.robotPoses.get(cameraIndex).size()]));
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + cameraName + "/RobotPosesAccepted",
                    this.robotPosesAccepted.get(cameraIndex)
                            .toArray(new Pose3d[this.robotPosesAccepted.get(cameraIndex).size()]));
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + cameraName + "/RobotPosesRejected",
                    this.robotPosesRejected.get(cameraIndex)
                            .toArray(new Pose3d[this.robotPosesRejected.get(cameraIndex).size()]));

            this.allRobotPoses.addAll(robotPoses.get(cameraIndex));
            this.allRobotPosesAccepted.addAll(robotPosesAccepted.get(cameraIndex));
            this.allRobotPosesRejected.addAll(robotPosesRejected.get(cameraIndex));
            this.allTagPoses.addAll(tagPoses.get(cameraIndex));
        }

        // Log summary data
        Logger.recordOutput("Subsystems/Vision/Summary/TagPoses",
                this.allTagPoses.toArray(new Pose3d[this.allTagPoses.size()]));
        Logger.recordOutput("Subsystems/Vision/Summary/RobotPoses",
                this.allRobotPoses.toArray(new Pose3d[this.allRobotPoses.size()]));
        Logger.recordOutput(
                "Subsystems/Vision/Summary/RobotPosesAccepted",
                this.allRobotPosesAccepted.toArray(new Pose3d[this.allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Subsystems/Vision/Summary/RobotPosesRejected",
                this.allRobotPosesRejected.toArray(new Pose3d[this.allRobotPosesRejected.size()]));

        // Record cycle time
        LoggedTracer.record(this.NAME);
    }

    /**
     * 
     */
    public int getBestTargetId(String cameraName) {
        for (int i = 0; i < this.visionModules.length; i++) {
            if (this.visionModules[i].getName() == cameraName) {
                return this.inputs[i].bestTargetId;
            }
        }

        return 0;
    }

    /**
     * 
     */
    public Pose2d getBestRobotPose() {
        if (mostRecentBestPoseTimestamp > Timer.getTimestamp() - VisionConstants.BEST_POSE_TIME_THRESHOLD_SECS) {
            return this.mostRecentBestPose.toPose2d();
        } else {
            return new Pose2d();
        }
    }

    /**
     * 
     */
    public Pose3d getBestTargetPose(String cameraName) {
        for (int i = 0; i < this.visionModules.length; i++) {
            if (this.visionModules[i].getName() == cameraName) {
                return this.inputs[i].bestTargetPose;
            }
        }

        return null;
    }

    /**
     * 
     */
    public Transform3d getRobotToCamera(String cameraName) {
        VisionModuleIO visionModule = getVisionModuleByName(cameraName);

        return visionModule == null ? null : visionModule.getRobotToCamera();
    }

    /**
     * 
     */
    public VisionModuleIO getVisionModuleByName(String cameraName) {
        for (int i = 0; i < this.visionModules.length; i++) {
            if (this.visionModules[i].getName() == cameraName) {
                return this.visionModules[i];
            }
        }

        return null;
    }

    /**
     * 
     */
    private boolean arePoseRotationsReasonable(Pose3d pose) {
        return Math.abs(
                this.poseSupplier.get()
                        .getRotation()
                        .minus(pose.getRotation().toRotation2d())
                        .getRadians()) < 10.0;
    }

    /**
     * Returns true if the specified pose is on the field (or within
     * FIELD_BORDER_MARGIN_METERS in the
     * x and y directions and MAX_Z_ERROR_METERS in the z direction).
     *
     * @param pose the pose to check
     * @return true if the specified pose is on the field (or very close to it)
     */
    private boolean poseIsOnField(Pose3d pose) {
        return pose.getX() > -0.5
                && pose.getX() < FieldConstants.LENGTH_METERS + 0.5
                && pose.getY() > -0.5
                && pose.getY() < FieldConstants.LENGTH_METERS + 0.5
                && Math.abs(pose.getZ()) < 0.25;
    }

    /**
     * 
     */
    private void logCameraTransforms(String cameraName, PoseObservation observation) {
        // this is the pose of the robot when centered on the reef face that faces the
        // driver station
        Pose3d cameraPose = observation.cameraPose();
        Transform3d robotToCameraTransform = cameraPose.minus(this.robotPoseForCalibration);

        Logger.recordOutput("Subsystems/" + NAME + "/" + cameraName + "/RobotToCameraTransform",
                robotToCameraTransform);
        Logger.recordOutput("Subsystems/" + NAME + "/" + cameraName + "/RobotToCameraPose",
                this.robotPoseForCalibration);
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    private Matrix<N3, N1> getStandardDeviations(int index, PoseObservation observation) {
        double xyStdDev = 0.08
                * Math.pow(observation.averageTagDistance(), 2.0)
                / observation.numTags()
                * VisionConstants.CAMERA_STD_DEV_FACTORS[index];

        // for multi-tag strategies, scale the standard deviation by the reprojection
        // error; for
        // single-tag, scale by the ambiguity
        if (observation.type() == PoseObservationType.MULTI_TAG) {
            xyStdDev *= (3.33 * observation.reprojectionError());
        } else {
            xyStdDev *= (5.0 * observation.averageAmbiguity());
        }

        // only trust the rotation component for multi-tag strategies; for single-tag,
        // set the standard
        // deviation to infinity
        double thetaStdDev = observation.type() == PoseObservationType.MULTI_TAG
                ? 0.1
                        * Math.pow(observation.averageTagDistance(), 2.0)
                        * (3.33 * observation.reprojectionError())
                        / observation.numTags()
                        * VisionConstants.CAMERA_STD_DEV_FACTORS[index]
                : Double.POSITIVE_INFINITY;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
