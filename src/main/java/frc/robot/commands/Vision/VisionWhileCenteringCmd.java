package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class VisionWhileCenteringCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    private final double targetID = 5;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 0.97; // in meters
    double tagAngle = 0;

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    public VisionWhileCenteringCmd(Limelight limelight, Swerve swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;

        this.strafePidController = new PIDController(0.7, 0, 0.045);
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

    @Override
    public void execute() {

        if (limelight.getAprilTagID() == targetID) {
            // at___ = AprilTag____

            double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
            Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
            double robotHeading = swerveDrive.getGyroYaw().getDegrees();
            double x = limelight.getX(); // degrees from LL to the tag
            double yawDeg = botPose[4]; // degrees from the LL to the tags yaw
            double Dis = -botPose3D.getZ(); // Distance from LL to tag

            double[] targetPoseValues = LimelightHelpers.getTargetPose_RobotSpace("");
            Translation2d targetPose_relBot = new Translation2d(targetPoseValues[0], targetPoseValues[1]);
            Translation2d targetPose_relField = targetPose_relBot.rotateBy(swerveDrive.getGyroYaw());

            System.out.println(yawDeg);

            double rotationToTag = robotHeading - tagAngle;

            double zDis = Dis * Math.cos(Math.toRadians(robotHeading)); // Distance from robot to tag in relation of the
            // field (Adjacent)

            double atDeg = yawDeg - x; // Difference from the LL to the tag
            double atXDis = zDis * (Math.tan(Math.toRadians(atDeg)));

            // Left/Right
            double translationDiff = targetPose_relField.getY() - desiredz;
            double strafeDiff = -targetPose_relField.getX();

            if (Math.abs(rotationToTag) > 1) { // Adjust tolerance as needed
                rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
                rotationPidOutput = rotationPidOutput * 1; // Speed multiplier
                rotationPos = false;
            } else {
                rotationPidOutput = 0;
                rotationPos = true;
            }

            if (Math.abs(strafeDiff) > 0.015) { // In meters
                strafePidOutput = strafePidController.calculate(strafeDiff, 0);
                strafePidOutput = strafePidOutput * 1; // Speed multiplier
                xPos = false;
            } else {
                strafePidOutput = 0;
                xPos = true;
            }

            if (Math.abs(translationDiff) > 0.015) { // In meters
                translationPidOutput = translationPidController.calculate(translationDiff, 0);
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