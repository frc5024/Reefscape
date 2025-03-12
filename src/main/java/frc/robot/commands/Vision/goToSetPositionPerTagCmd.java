package frc.robot.commands.Vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class goToSetPositionPerTagCmd extends Command {
    static ShuffleboardTab tab = Shuffleboard.getTab("Tags");
    // GenericEntry pEntry = tab.add("SET P VISION", 0.7).getEntry();
    // GenericEntry iEntry = tab.add("SET I VISION", 0).getEntry();
    // GenericEntry dEntry = tab.add("SET D VISION", 0.05).getEntry();

    private final Limelight limelight;
    private final Swerve swerveDrive;
    private final double xOffset;
    // private LEDs s_LEDs;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 0.430; // in meters
    double desiredx = 0; // in meters? (+ right)?

    double tagAngle = 0;
    private final double cameraAngle = 29;

    boolean isLEDset = false;

    public goToSetPositionPerTagCmd(Limelight limelight, Swerve swerveDrive, double xOffset) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.xOffset = xOffset;

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
        // isLEDset = false;
    }

    Set<Integer> validTagIDs = new HashSet<>(
            Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        if (validTagIDs.contains(detectedTagID)) {
            swerveDrive.setFieldRelative(false);

            // if (!isLEDset) {
            // isLEDset = true;
            // s_LEDs.setCommand(LEDPreset.Solid.kWhite).schedule();
            // }

            mathToTag();
        } else {
            swerveDrive.visionTranslationalVal(0, false);
            swerveDrive.visionStrafeVal(0, false);
            swerveDrive.visionRotationVal(0, false);

            swerveDrive.setFieldRelative(true);
        }

    }

    public void mathToTag() {
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");

        double yaw = botPose[4] - cameraAngle;

        // SmartDashboard.putNumber("Tag Yaw", yaw);
        // SmartDashboard.putNumber("getZ botPose", botPose3D.getZ());

        // Left/Right
        double zDiff = botPose3D.getZ() + desiredz;

        // forward/back
        double xDiff = botPose3D.getX() - xOffset;

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        setDrive();
    }

    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > 1.5) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * 2.2; // Speed multiplier
            limelight.setRotationPos(false);
        } else {
            rotationPidOutput = 0;
            limelight.setRotationPos(true);
        }
        // SmartDashboard.putNumber("thetaDiff", rotationToTag);
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
        // SmartDashboard.putNumber("zDiff", zDiff);
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
        // SmartDashboard.putNumber("xDiff", xDiff);
    }

    public void setDrive() {
        swerveDrive.setFieldRelative(false);

        // SmartDashboard.putNumber("StrafePID", strafePidOutput);
        // SmartDashboard.putNumber("TranslatPID", translationPidOutput);

        swerveDrive.visionRotationVal(rotationPidOutput, true);
        swerveDrive.visionTranslationalVal(translationPidOutput, true);
        swerveDrive.visionStrafeVal(strafePidOutput, true);
    }

    @Override
    public boolean isFinished() {
        return limelight.getXPos() && limelight.getZPos() && limelight.getRotationPos(); // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);

        swerveDrive.setFieldRelative(true);

        // swerveDrive.setPose(swerveDrive.getPose());
    }
}