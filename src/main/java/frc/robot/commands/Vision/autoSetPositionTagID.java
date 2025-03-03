package frc.robot.commands.Vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class autoSetPositionTagID extends Command {
    static ShuffleboardTab tab = Shuffleboard.getTab("Tags");
    // GenericEntry pEntry = tab.add("SET P VISION", 0.7).getEntry();
    // GenericEntry iEntry = tab.add("SET I VISION", 0).getEntry();
    // GenericEntry dEntry = tab.add("SET D VISION", 0.05).getEntry();

    private final Limelight limelight;
    private final Swerve swerveDrive;
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

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    double tag = 0;
    double validTagID;

    public autoSetPositionTagID(Limelight limelight, Swerve swerveDrive, double xOffset, double tag) {
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

        xPos = false;
        zPos = false;
        rotationPos = false;

        lastSeenRotation = 0;
        lastSeenStrafe = 0;
    }

    Set<Integer> tag6 = new HashSet<>(
            Set.of(6, 19));

    Set<Integer> tag7 = new HashSet<>(
            Set.of(7, 18));

    Set<Integer> tag8 = new HashSet<>(
            Set.of(8, 17));

    Set<Integer> tag9 = new HashSet<>(
            Set.of(9, 22));

    Set<Integer> tag10 = new HashSet<>(
            Set.of(10, 21));

    Set<Integer> tag11 = new HashSet<>(
            Set.of(11, 20));

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        if (tag == detectedTagID) {
            swerveDrive.setFieldRelative(false);

            if (strafePidOutput != 0) {
                lastSeenStrafe = strafePidOutput;
            }

            if (rotationPidOutput != 0) {
                lastSeenRotation = rotationPidOutput;
            }

            mathToTag();
        } else {
            swerveDrive.visionRotationVal(lastSeenRotation, true);
            swerveDrive.visionStrafeVal(lastSeenStrafe, true);
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

        SmartDashboard.putNumber("Tag Yaw", yaw);

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        setDrive();
    }

    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > 1) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * 1.5; // Speed multiplier
            // if (rotationPidOutput > 2)
            // rotationPidOutput = 2; // check values at certain distance to find good speed
            // for all below
            rotationPos = false;
        } else {
            rotationPidOutput = 0;
            rotationPos = true;
        }
        SmartDashboard.putNumber("thetaDiff", rotationToTag);
        SmartDashboard.putNumber("RotationPid", rotationPidOutput);
    }

    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > 0.06) { // In meters
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = translationPidOutput * 1; // Speed multiplier
            // if (translationPidOutput > 2)
            // translationPidOutput = 2;
            zPos = false;
        } else {
            translationPidOutput = 0;
            zPos = true;
        }
        SmartDashboard.putNumber("zDiff", zDiff);
        SmartDashboard.putNumber("TranslatPID", translationPidOutput);
    }

    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > 0.025) { // In meters
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * 1; // Speed multiplier
            // if (strafePidOutput > 2)
            // strafePidOutput = 2;
            xPos = false;
        } else {
            strafePidOutput = 0;
            xPos = true;
        }
        SmartDashboard.putNumber("xDiff", xDiff);
        SmartDashboard.putNumber("StrafePID", strafePidOutput);
    }

    public void setDrive() {
        swerveDrive.setFieldRelative(false);

        System.out.println("driving");

        SmartDashboard.putBoolean("rotationPos", rotationPos);
        SmartDashboard.putBoolean("xPos", xPos);
        SmartDashboard.putBoolean("zPos", zPos);

        swerveDrive.visionRotationVal(rotationPidOutput, true);
        swerveDrive.visionTranslationalVal(translationPidOutput, true);
        swerveDrive.visionStrafeVal(strafePidOutput, true);

        swerveDrive.drive(true);
    }

    @Override
    public boolean isFinished() {
        return xPos && zPos && rotationPos; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);

        swerveDrive.setFieldRelative(true);

        swerveDrive.resetSwerve();
        swerveDrive.setPose(Pose2d.kZero);

        if (xPos && zPos && rotationPos) {
            System.out.println("amcoansocnasasdacs");
        }
    }
}