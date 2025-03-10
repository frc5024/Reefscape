package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class autoSetPositionTagID extends Command {
    static ShuffleboardTab tab = Shuffleboard.getTab("Tags");
    // GenericEntry pEntry = tab.add("SET P VISION", 0.7).getEntry();
    // GenericEntry iEntry = tab.add("SET I VISION", 0).getEntry();
    // GenericEntry dEntry = tab.add("SET D VISION", 0.05).getEntry();

    private final Limelight limelight;
    private final SwerveDriveSubsystem swerveDrive;
    private final double xOffset;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    double lastSeenStrafe = 0;
    double lastSeenRotation = 0;
    double lastSeenTranslation = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 0.430; // in meters
    double desiredx = 0; // in meters? (+ right)?

    double tagAngle = 0;
    private final double cameraAngle = 29;

    double tag = 0;
    double tag2 = 0;
    double validTagID;

    public autoSetPositionTagID(Limelight limelight, SwerveDriveSubsystem swerveDrive, double xOffset, double tag) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.xOffset = xOffset;
        this.tag = tag;

        this.strafePidController = new PIDController(0.7, 0, 0.05);
        this.translationPidController = new PIDController(0.5, 0, 0.05);
        this.rotationPidController = new PIDController(0.008, 0, 0.0005);

    }

    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();

        limelight.setRotationPos(false);
        limelight.setXPos(false);
        limelight.setZPos(false);

        if (tag == 11) {
            tag2 = 20;
        }
        if (tag == 10) {
            tag2 = 21;
        }
        if (tag == 9) {
            tag2 = 22;
        }
        if (tag == 8) {
            tag2 = 17;
        }
        if (tag == 7) {
            tag2 = 18;
        }
        if (tag == 6) {
            tag2 = 19;
        }

        // lastSeenRotation = 0;
        // lastSeenStrafe = 0;
    }

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        SmartDashboard.putNumber("Goal Tag Red Side (auto)", tag);
        SmartDashboard.putNumber("Goal Tag Blue Side (auto)", tag2);

        if (detectedTagID == tag || detectedTagID == tag2) {
            swerveDrive.setIsFieldRelative(false);

            // if (strafePidOutput != 0) {
            // lastSeenStrafe = strafePidOutput;
            // }

            // if (rotationPidOutput != 0) {
            // lastSeenRotation = rotationPidOutput;
            // }

            mathToTag();
        } else {
            // swerveDrive.visionRotationVal(lastSeenRotation, true);
            // swerveDrive.visionStrafeVal(lastSeenStrafe, true);

            // swerveDrive.visionTranslationalVal(0, false);
            // swerveDrive.visionStrafeVal(0, false);
            // swerveDrive.visionRotationVal(0, false);

            swerveDrive.setIsFieldRelative(true);
        }

    }

    public void mathToTag() {
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");

        // Rotation
        double yaw = botPose[4] - cameraAngle;

        // Left/Right
        double zDiff = botPose3D.getZ() + desiredz;

        // forward/back
        double xDiff = botPose3D.getX() - xOffset;

        // SmartDashboard.putNumber("Tag Yaw", yaw);

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        setDrive();
    }

    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > 1.5) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * 2; // Speed multiplier
            limelight.setRotationPos(false);
        } else {
            rotationPidOutput = 0;
            limelight.setRotationPos(true);
        }
    }

    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > 0.08) { // In meters
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = translationPidOutput * 2.6; // Speed multiplier (1.2)
            if (translationPidOutput > 0.3)
                translationPidOutput = 0.3;
            limelight.setZPos(false);
        } else {
            translationPidOutput = 0;
            limelight.setZPos(true);
        }
    }

    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > 0.025) { // In meters
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * 2.3; // Speed multiplier
            if (strafePidOutput > 0.15)
                strafePidOutput = 0.15;
            if (strafePidOutput < -0.15)
                strafePidOutput = -0.15;

            limelight.setXPos(false);
        } else {
            strafePidOutput = 0;
            limelight.setXPos(true);
        }
    }

    public void setDrive() {
        swerveDrive.setIsFieldRelative(false);

        // swerveDrive.visionRotationVal(rotationPidOutput, true);
        // swerveDrive.visionTranslationalVal(translationPidOutput, true);
        // swerveDrive.visionStrafeVal(strafePidOutput, true);

        // swerveDrive.drive(true);
    }

    @Override
    public boolean isFinished() {
        return limelight.getXPos() && limelight.getZPos() && limelight.getRotationPos(); // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        // swerveDrive.visionTranslationalVal(0, false);
        // swerveDrive.visionStrafeVal(0, false);
        // swerveDrive.visionRotationVal(0, false);

        tag = 0;
        tag2 = 0;

        swerveDrive.setIsFieldRelative(true);

        // swerveDrive.resetSwerve();
    }
}