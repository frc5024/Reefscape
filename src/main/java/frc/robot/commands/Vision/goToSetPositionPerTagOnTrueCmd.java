package frc.robot.commands.Vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class goToSetPositionPerTagOnTrueCmd extends Command {

    private final Limelight limelight;
    private final Swerve swerveDrive;
    private final double xOffset;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 1; // in meters
    double desiredx = 0; // in meters? (+ right)?

    double tagAngle = 0;

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    double speedMultiplier;

    public goToSetPositionPerTagOnTrueCmd(Limelight limelight, Swerve swerveDrive, double xOffset) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.xOffset = xOffset;

        this.strafePidController = new PIDController(0.7, 0, 0.05);
        this.translationPidController = new PIDController(0.7, 0, 0.05);
        this.rotationPidController = new PIDController(0.008, 0, 0.0005);

        addRequirements(swerveDrive); // might not work now but should fix errors
    }

    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();
    }

    Set<Integer> validTagIDs = new HashSet<>(
            Set.of(1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21, 22));
    // 3, 16 - processor
    // 1, 2, 12, 13 - coral station
    // 6, 7, 8, 9, 10, 11 - red reef
    // 17, 18, 19, 20, 21, 22 - blue reef

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        // if robot rotates long ways, add the angle to the heading in
        // a new variable to lie to the robot on which will be the quickest rotation

        if (validTagIDs.contains(detectedTagID)) {
            swerveDrive.setFieldRelative(false);

            // tag angle = angle based on the ROBOTS forward/heading
            if (detectedTagID == 18 || detectedTagID == 7) {
                tagAngle = 0;
            } else if (detectedTagID == 10 || detectedTagID == 21) {
                tagAngle = -179.5;
            } else if (detectedTagID == 3 || detectedTagID == 16) {
                tagAngle = 90;
            } else {
                tagAngle = 0;
            }

            mathToTag();
        } else {
            swerveDrive.visionTranslationalVal(0, false);
            swerveDrive.visionStrafeVal(0, false);
            swerveDrive.visionRotationVal(0, false);

            swerveDrive.setFieldRelative(true);

            cancel();
        }

    }

    public void mathToTag() {
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
        double robotHeading = swerveDrive.getGyroYaw().getDegrees();
        // double robotHeading360 = swerveDrive.getGyroYaw360().getDegrees();
        double x = limelight.getX();
        double yawDeg = botPose[4];
        double Dis = -botPose3D.getZ(); // Distance from LL to tag

        double rotationToTag = robotHeading + tagAngle;

        double zDis = Dis * Math.cos(Math.toRadians(rotationToTag)); // Distance from robot to tag in relation of the
                                                                     // field

        double atDeg = yawDeg - x; // might want to switch to 3D X value
        double xDis = zDis * (Math.tan(Math.toRadians(atDeg))); // might want to switch to rotation to tag instead of
                                                                // atdeg

        // Left/Right
        double zDiff = zDis - desiredz;

        // forward/back
        double xDiff = xDis + xOffset;

        // if (elevator up) { half speed }
        speedMultiplier = 1;

        rotateToTag(rotationToTag);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        setDrive();
    }

    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > 1) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * speedMultiplier; // Speed multiplier
            rotationPos = false;
        } else {
            rotationPidOutput = 0;
            rotationPos = true;
        }
    }

    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > 0.015) { // In meters
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = -translationPidOutput * speedMultiplier; // Speed multiplier
            zPos = false;
        } else {
            translationPidOutput = 0;
            zPos = true;
        }
    }

    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > 0.015) { // In meters
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * speedMultiplier; // Speed multiplier
            xPos = false;
        } else {
            strafePidOutput = 0;
            xPos = true;
        }
    }

    public void setDrive() {
        swerveDrive.setFieldRelative(false);

        swerveDrive.visionRotationVal(rotationPidOutput, true);
        swerveDrive.visionTranslationalVal(translationPidOutput, true);
        swerveDrive.visionStrafeVal(strafePidOutput, true);

        // if within certain range set the elevator to start

        if (xPos && zPos && rotationPos) {
            // coralOuttake();

            cancel();
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

        swerveDrive.setFieldRelative(true);
    }
}