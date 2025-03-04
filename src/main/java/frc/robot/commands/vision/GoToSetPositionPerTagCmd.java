package frc.robot.commands.vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class GoToSetPositionPerTagCmd extends Command {
    static ShuffleboardTab tab = Shuffleboard.getTab("Tags");
    // GenericEntry pEntry = tab.add("SET P VISION", 0.7).getEntry();
    // GenericEntry iEntry = tab.add("SET I VISION", 0).getEntry();
    // GenericEntry dEntry = tab.add("SET D VISION", 0.05).getEntry();

    private final Limelight limelight;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final double xOffset;

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

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    public GoToSetPositionPerTagCmd(Limelight limelight, SwerveDriveSubsystem swerveDriveSubsystem, double xOffset) {
        this.limelight = limelight;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
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

        xPos = false;
        zPos = false;
        rotationPos = false;
    }

    Set<Integer> validTagIDs = new HashSet<>(
            Set.of(2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21, 22));
    // 3, 16 - processor
    // 1, 2, 12, 13 - coral station
    // 6, 7, 8, 9, 10, 11 - red reef
    // 17, 18, 19, 20, 21, 22 - blue reef

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        if (validTagIDs.contains(detectedTagID)) {
            swerveDriveSubsystem.setIsFieldRelative(false);

            mathToTag();
        } else {
            // swerveDriveSubsystem.visionTranslationalVal(0, false);
            // swerveDriveSubsystem.visionStrafeVal(0, false);
            // swerveDriveSubsystem.visionRotationVal(0, false);

            swerveDriveSubsystem.setIsFieldRelative(true);
            swerveDriveSubsystem.visionSlowMode = false;
        }

    }

    public void mathToTag() {
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");

        double yaw = botPose[4] - cameraAngle;

        SmartDashboard.putNumber("Tag Yaw", yaw);

        // Left/Right
        double zDiff = botPose3D.getZ() + desiredz;

        // forward/back
        double xDiff = botPose3D.getX() - xOffset;

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        swerveDriveSubsystem.visionSlowMode = true;
        setDrive();
    }

    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > 1) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * 2; // Speed multiplier
            rotationPos = false;
        } else {
            rotationPidOutput = 0;
            rotationPos = true;
        }
        SmartDashboard.putNumber("thetaDiff", rotationToTag);
    }

    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > 0.06) { // In meters
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = translationPidOutput * 1.7; // Speed multiplier
            zPos = false;
        } else {
            translationPidOutput = 0;
            zPos = true;
        }
        SmartDashboard.putNumber("zDiff", zDiff);
    }

    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > 0.025) { // In meters
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * 1.4; // Speed multiplier
            xPos = false;
        } else {
            strafePidOutput = 0;
            xPos = true;
        }
        SmartDashboard.putNumber("xDiff", xDiff);
    }

    public void setDrive() {
        swerveDriveSubsystem.setIsFieldRelative(false);

        SmartDashboard.putBoolean("rotationPos", rotationPos);
        SmartDashboard.putBoolean("xPos", xPos);
        SmartDashboard.putBoolean("zPos", zPos);

        // swerveDriveSubsystem.visionRotationVal(rotationPidOutput, true);
        // swerveDriveSubsystem.visionTranslationalVal(translationPidOutput, true);
        // swerveDriveSubsystem.visionStrafeVal(strafePidOutput, true);
    }

    @Override
    public boolean isFinished() {
        return xPos && zPos && rotationPos; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        // swerveDriveSubsystem.visionTranslationalVal(0, false);
        // swerveDriveSubsystem.visionStrafeVal(0, false);
        // swerveDriveSubsystem.visionRotationVal(0, false);

        swerveDriveSubsystem.setIsFieldRelative(true);
        swerveDriveSubsystem.visionSlowMode = false;

        // swerveDrive.setPose(swerveDrive.getPose());

        if (xPos && zPos && rotationPos) {
            System.out.println("amcoansocnasasdacs");
        }
    }
}