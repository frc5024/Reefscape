package frc.robot.commands.Vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.Swerve;

public class goToSetPositionPerTagCmd extends Command {

    private final Limelight limelight;
    private final Swerve swerveDrive;
    private final double xOffset;
    Rumble rumble = Rumble.getInstance();
    // private LEDs s_LEDs;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    // boolean shouldBeSlowDis = false;
    // boolean shouldBeSlowStrafe = false;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 0.40; // in meters

    double tagAngle = 0;
    private final double cameraAngle = 27.5;

    boolean isLEDset = false;

    private Command ledCmd;
    private Command ledOffCmd;

    public goToSetPositionPerTagCmd(Limelight limelight, Swerve swerveDrive, double xOffset) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.xOffset = xOffset;

        strafePidController = new PIDController(0.7, 0, 0.05);
        translationPidController = new PIDController(0.5, 0, 0.05);
        rotationPidController = new PIDController(0.008, 0, 0.0005);

        ledCmd = LEDs.getInstance().persistCommand(LEDPreset.Solid.kHotPink);
        ledOffCmd = LEDs.getInstance().persistCommand(LEDPreset.Solid.kBlue);
    }

    // clarifies that the robot is NOT in position to avoid previous use conflicts
    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();

        // shouldBeSlowDis = false;
        // shouldBeSlowStrafe = false;

        limelight.isVisionActivated(false);

        limelight.setRotationPos(false);
        limelight.setXPos(false);
        limelight.setZPos(false);

    }

    // Hashset of reef/wanted AprilTag IDs
    Set<Integer> validTagIDs = new HashSet<>(
            Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        // checks to see if detected tag is apart of the HashSet
        if (validTagIDs.contains(detectedTagID)) {
            swerveDrive.setFieldRelative(false);

            ledCmd.schedule();
            ledOffCmd.cancel();

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

            ledCmd.cancel();
            ledOffCmd.schedule();
        }

    }

    public void mathToTag() {
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");

        double yaw = botPose[4] - cameraAngle;

        // Left/Right
        double zDiff = botPose3D.getZ() + desiredz;

        // forward/back
        double xDiff = botPose3D.getX() - xOffset;

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        setDrive();
    }

    // Calculates and outpits PID based on difference to goal state (rotations)
    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > Constants.Vision.rotationTolerance) {
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * Constants.Vision.rotationPIDMultiplier; // PID multiplier
            limelight.setRotationPos(false);
        } else {
            rotationPidOutput = 0;
            limelight.setRotationPos(true);
        }

        limelight.setRotationDiff(rotationToTag);
    }

    // Calculates and outpits PID based on difference to goal state (Forward
    // distance)
    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > Constants.Vision.distanceTolerance) {
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = translationPidOutput * Constants.Vision.distancePIDMultiplier; // PID multiplier
            if (translationPidOutput > Constants.Vision.distancePIDCap) // PID/Speed cap
                translationPidOutput = Constants.Vision.distancePIDCap;
            limelight.setZPos(false);
        } else {
            translationPidOutput = 0;
            limelight.setZPos(true);
        }

        limelight.setDistanceDiff(zDiff);
        // if (Math.abs(zDiff) < 0.4) {
        // shouldBeSlowDis = true;
        // } else {
        // shouldBeSlowDis = false;
        // }
    }

    // Calculates and outpits PID based on difference to goal state (Left/Right)
    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > Constants.Vision.strafeTolerance) {
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * Constants.Vision.strafePIDMultiplier; // PID multiplier
            if (strafePidOutput > Constants.Vision.strafePIDCap) // PID/Speed cap
                strafePidOutput = Constants.Vision.strafePIDCap;
            if (strafePidOutput < -Constants.Vision.strafePIDCap)
                strafePidOutput = -Constants.Vision.strafePIDCap;

            limelight.setXPos(false);
        } else {
            strafePidOutput = 0;
            limelight.setXPos(true);
        }
        limelight.setStrafeDiff(xDiff);

        // if (Math.abs(xDiff) < 0.2) {
        // shouldBeSlowStrafe = true;
        // } else {
        // shouldBeSlowStrafe = false;
        // }
    }

    // PID/Speed cap increase smaller adjustments within the PID but caps the max
    // allowing for slower/controlled overall speed but larger smaller adjustments

    public void setDrive() {
        swerveDrive.setFieldRelative(false);

        // if (shouldBeSlowDis && shouldBeSlowStrafe) {
        // swerveDrive.isSlowMode = true;
        // } else {
        // swerveDrive.isSlowMode = false;
        // }

        swerveDrive.visionRotationVal(rotationPidOutput, true);
        swerveDrive.visionTranslationalVal(translationPidOutput, true);
        swerveDrive.visionStrafeVal(strafePidOutput, true);
    }

    @Override
    public boolean isFinished() {
        return limelight.inPosition(); // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.isSlowMode = false;

        // shouldBeSlowDis = false;
        // shouldBeSlowStrafe = false;

        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);

        limelight.isVisionActivated(true);

        swerveDrive.setFieldRelative(true);

        ledCmd.cancel();
        ledOffCmd.cancel();

        if (!interrupted) {
            LEDs.getInstance().flashCommand(LEDPreset.Solid.kHotPink, 2).schedule();
            rumble.staticRumble(true);
        }

        // swerveDrive.setPose(swerveDrive.getPose());
    }
}