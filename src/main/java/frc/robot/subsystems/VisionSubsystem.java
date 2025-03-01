package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.camera.Camera;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.modules.vision.VisionIOInputsAutoLogged;
import frc.robot.modules.vision.VisionModuleIO;
import frc.robot.modules.vision.VisionModuleIO.PoseObservationType;
import frc.robot.modules.vision.VisionModuleIOLimelight;
import frc.robot.modules.vision.VisionModuleIOPhotonVision;
import frc.robot.modules.vision.VisionModuleIOPhotonVisionSim;

public class VisionSubsystem extends SubsystemBase {
    /* Modules */
    private List<VisionModuleIO> visionModules = new ArrayList<VisionModuleIO>();

    /* Variables */
    private final VisionConsumer visionConsumer;
    private final Supplier<Pose2d> poseSupplier;
    private final HashMap<String, VisionIOInputsAutoLogged> inputs;
    private final HashMap<String, Alert> disconnectedAlerts;

    private final List<Pose3d> allTagPoses = new LinkedList<>();
    private final List<Pose3d> allRobotPoses = new LinkedList<>();
    private final List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    private final List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    /**
     * 
     */
    public VisionSubsystem(VisionConsumer visionConsumer, Supplier<Pose2d> poseSupplier,
            Supplier<Rotation2d> rotationSupplier) {
        this.visionConsumer = visionConsumer;
        this.poseSupplier = poseSupplier;

        // Setup the vision modules
        for (Camera camera : VisionConstants.CAMERAS) {
            if (Robot.isReal()) {
                if (camera.getProcessor() == Camera.Processor.LIMELIGHT) {
                    this.visionModules.add(new VisionModuleIOLimelight(camera, rotationSupplier));
                } else if (camera.getProcessor() == Camera.Processor.PHOTONVISION) {
                    this.visionModules.add(new VisionModuleIOPhotonVision(camera));
                }
            } else {
                if (camera.getProcessor() == Camera.Processor.LIMELIGHT) {
                    this.visionModules.add(new VisionModuleIOLimelight(camera, rotationSupplier));
                } else if (camera.getProcessor() == Camera.Processor.PHOTONVISION) {
                    // this.visionModules.add(new VisionModuleIOPhotonVision(camera));
                    this.visionModules.add(new VisionModuleIOPhotonVisionSim(camera, this.poseSupplier));
                }
            }
        }

        // Initialize inputs
        this.inputs = new HashMap<>();
        for (VisionModuleIO visionModuleIO : this.visionModules) {
            this.inputs.put(visionModuleIO.getName(), new VisionIOInputsAutoLogged());
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new HashMap<>();
        for (VisionModuleIO visionModuleIO : this.visionModules) {
            this.disconnectedAlerts.put(visionModuleIO.getName(),
                    new Alert("Vision camera is disconnected.", AlertType.kWarning));
        }
    }

    @Override
    public void periodic() {
        for (VisionModuleIO visionModuleIO : this.visionModules) {
            String name = visionModuleIO.getName();
            VisionIOInputsAutoLogged inputs = this.inputs.get(name);

            visionModuleIO.updateInputs(inputs);
            Logger.processInputs("Vision/Camera " + name, inputs);
        }

        // Initialize logging values
        this.allTagPoses.clear();
        this.allRobotPoses.clear();
        this.allRobotPosesAccepted.clear();
        this.allRobotPosesRejected.clear();

        // Loop over cameras
        int cameraIndex = 0;
        for (VisionModuleIO visionModuleIO : this.visionModules) {
            // Update disconnected alert
            String name = visionModuleIO.getName();
            VisionIOInputsAutoLogged inputs = this.inputs.get(name);

            Alert disconnectedAlert = this.disconnectedAlerts.get(name);
            disconnectedAlert.set(!inputs.connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs.tagIds) {
                var tagPose = VisionConstants.TAG_FIELD_LAYOUT.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs.poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)
                        || Math.abs(observation.pose().getZ()) > VisionConstants.APRILTAG_MAX_Z_ERROR
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > VisionConstants.TAG_FIELD_LAYOUT.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > VisionConstants.TAG_FIELD_LAYOUT.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= VisionConstants.LINEAR_STD_DEV_MEGATAG2_FACTOR;
                    angularStdDev *= VisionConstants.ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                }
                if (cameraIndex < VisionConstants.CAMERA_STD_DEV_FACTORS.length) {
                    linearStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
                    angularStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
                }

                // Send vision observation
                this.visionConsumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + name + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + name + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + name + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Subsystems/Vision/Camera " + name + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            Logger.recordOutput("Subsystems/Vision/Camera " + name + "/Tags", inputs.tagIds);

            this.allTagPoses.addAll(tagPoses);
            this.allRobotPoses.addAll(robotPoses);
            this.allRobotPosesAccepted.addAll(robotPosesAccepted);
            this.allRobotPosesRejected.addAll(robotPosesRejected);

            cameraIndex++;
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
    }

    /**
     * 
     */
    public int getBestTargetId(String cameraName) {
        VisionIOInputsAutoLogged inputs = this.inputs.get(cameraName);

        if (inputs == null)
            return 0;

        return inputs.bestTargetId;
    }

    /**
     * 
     */
    public Pose2d getBestRobotPose() {
        return this.allRobotPosesAccepted.size() > 0 ? this.allRobotPosesAccepted.get(0).toPose2d() : new Pose2d();
    }

    /**
     * 
     */
    public Pose3d getBestTargetPose(String cameraName) {
        VisionIOInputsAutoLogged inputs = this.inputs.get(cameraName);

        if (inputs == null) {
            return null;
        }

        if (inputs.processor != Camera.Processor.LIMELIGHT) {
            return inputs.bestTargetPose;
        }

        // Limelight best target pose is relative to the robot
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        Pose3d targetPose = inputs.bestTargetPose;
        return robotPose.transformBy(
                new Transform3d(targetPose.getZ(), targetPose.getX(), robotPose.getZ(),
                        targetPose.getRotation().rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)))));
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
        if (this.visionModules.isEmpty())
            return null;

        VisionModuleIO visionModule = this.visionModules.stream()
                .filter(module -> module.getName() == cameraName)
                .findFirst()
                .get();

        return visionModule;
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
