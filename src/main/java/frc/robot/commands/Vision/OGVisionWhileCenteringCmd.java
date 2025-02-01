package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class OGVisionWhileCenteringCmd extends Command {
    static ShuffleboardTab tab = Shuffleboard.getTab("Tag");
    GenericEntry pEntry = tab.add("SET P", 0.7).getEntry();
    GenericEntry dEntry = tab.add("SET D", 0.05).getEntry();
    GenericEntry iEntry = tab.add("SET I", 0).getEntry();

    private final Limelight limelight;
    private final Swerve swerveDrive;

    private final double targetID = 5;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 1; // in meters
    double tagAngle = 0;

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    public OGVisionWhileCenteringCmd(Limelight limelight, Swerve swerveDrive) {
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

    @Override
    public void execute() {

        if (limelight.getAprilTagID() == targetID) {
            // at___ = AprilTag____

            translationPidController.setP(pEntry.getDouble(0));
            translationPidController.setI(iEntry.getDouble(0));
            translationPidController.setD(dEntry.getDouble(0));

            strafePidController.setP(pEntry.getDouble(0));
            strafePidController.setI(iEntry.getDouble(0));
            strafePidController.setD(dEntry.getDouble(0));

            double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
            Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
            double robotHeading = swerveDrive.getGyroYaw().getDegrees();
            double robotHeading360 = swerveDrive.getGyroYaw360().getDegrees();
            double x = limelight.getX();
            double yawDeg = botPose[4];
            double Dis = -botPose3D.getZ(); // Distance from LL to tag

            double rotationToTag = robotHeading - tagAngle;

            double zDis = Dis * Math.cos(Math.toRadians(robotHeading)); // Distance from robot to tag in relation of the
                                                                        // field

            double atDeg = yawDeg - x;
            double atXDis = zDis * (Math.tan(Math.toRadians(atDeg)));

            // Left/Right
            double zDiff = desiredz - zDis;

            if (Math.abs(rotationToTag) > 1) { // Adjust tolerance as needed
                rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
                rotationPidOutput = rotationPidOutput * 1; // Speed multiplier
                rotationPos = false;
            } else {
                rotationPidOutput = 0;
                rotationPos = true;
            }

            if (Math.abs(atXDis) > 0.015) { // In meters
                strafePidOutput = strafePidController.calculate(atXDis, 0);
                strafePidOutput = -strafePidOutput * 1; // Speed multiplier
                xPos = false;
            } else {
                strafePidOutput = 0;
                xPos = true;
            }

            if (Math.abs(zDiff) > 0.015) { // In meters
                translationPidOutput = translationPidController.calculate(zDiff, 0);
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