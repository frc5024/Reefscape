package frc.robot.commands.Vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class UpdatedWhileCenteringCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 0.97; // in meters

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    public UpdatedWhileCenteringCmd(Limelight limelight, Swerve swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;

        this.strafePidController = new PIDController(0.7, 0, 0.05);
        this.translationPidController = new PIDController(0.7, 0, 0.05);
        this.rotationPidController = new PIDController(0.008, 0, 0.0005);

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();
    }

    Set<Integer> validTagIDs = new HashSet<>(
            Set.of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22));

    @Override
    public void execute() {
        double detectedTagID = limelight.getAprilTagID();
        double tagAngle = 0;

        if (detectedTagID == 5) {
            tagAngle = 0;
        } else if (detectedTagID == 7) {
            tagAngle = -30;
        }

        if (detectedTagID == 5) {
            double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
            Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
            double robotHeading = swerveDrive.getGyroYaw().getDegrees();
            double x = limelight.getX();
            double yawDeg = botPose[4];
            double Dis = -botPose3D.getZ();

            double zDis = Dis * Math.cos(Math.toRadians(robotHeading));

            double atDeg = yawDeg - x;
            double atRad = Math.toRadians(atDeg);
            double atXDis = zDis * (Math.tan(atRad));

            double zDiff = desiredz - zDis;

            double tagYawRad = Math.toRadians(yawDeg); // Convert tag yaw to radians

            double tagRotation = robotHeading + tagAngle;

            // Convert robot's desired movement into the tag's coordinate system
            double tagRelativeX = atXDis * Math.cos(tagYawRad) + zDis * Math.sin(tagYawRad);
            double tagRelativeZ = zDis * Math.cos(tagYawRad) - atXDis * Math.sin(tagYawRad);

            if (Math.abs(tagRotation) > 1) { // Adjust tolerance as needed
                rotationPidOutput = rotationPidController.calculate(tagRotation, 0);
                rotationPidOutput = rotationPidOutput * 1; // Speed multiplier
                rotationPos = false;
            } else {
                rotationPidOutput = 0;
                rotationPos = true;
            }

            if (Math.abs(atXDis) > 0.015) { // In meters
                strafePidOutput = strafePidController.calculate(tagRelativeX, 0);
                strafePidOutput = -strafePidOutput * 1; // Speed multiplier
                xPos = false;
            } else {
                strafePidOutput = 0;
                xPos = true;
            }

            if (Math.abs(zDiff) > 0.015) { // In meters
                translationPidOutput = translationPidController.calculate(tagRelativeZ, 0);
                translationPidOutput = translationPidOutput * 1; // Speed multiplier
                zPos = false;
            } else {
                translationPidOutput = 0;
                zPos = true;
            }

            swerveDrive.visionStrafeVal(strafePidOutput, true);
            swerveDrive.visionRotationVal(rotationPidOutput, true);
            swerveDrive.visionTranslationalVal(translationPidOutput, true);
        } else {
            swerveDrive.visionTranslationalVal(0, false);
            swerveDrive.visionStrafeVal(0, false);
            swerveDrive.visionRotationVal(0, false);

        }
    }

    @Override
    public boolean isFinished() {
        return false; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);
    }
}

// potential upgrade; once yaw hits within 0 degree range 3 times in a row
// ignore all inputs until degree has reached 3 degrees 3 times in a row