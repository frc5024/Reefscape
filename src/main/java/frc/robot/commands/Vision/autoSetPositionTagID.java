package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class autoSetPositionTagID extends Command {

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

    double tagAngle = 0;
    private final double cameraAngle = 29;

    double tag = 0;
    double tag2 = 0;
    double validTagID;

    private Command ledCmd;
    private Command ledOffCmd;

    public autoSetPositionTagID(Limelight limelight, Swerve swerveDrive, double xOffset, double tag) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.xOffset = xOffset;
        this.tag = tag;

        strafePidController = new PIDController(0.7, 0, 0.05);
        translationPidController = new PIDController(0.5, 0, 0.05);
        rotationPidController = new PIDController(0.008, 0, 0.0005);

        ledCmd = LEDs.getInstance().persistCommand(LEDPreset.Solid.kHotPink);
        ledOffCmd = LEDs.getInstance().persistCommand(LEDPreset.Solid.kBlue);
    }

    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();

        limelight.setRotationPos(false);
        limelight.setXPos(false);
        limelight.setZPos(false);

        // set viable tags by comparing mirroded IDs
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

    // if wanted tag than lock controller
    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        if (detectedTagID == tag || detectedTagID == tag2) {
            swerveDrive.setFieldRelative(false);

            // if (strafePidOutput != 0) {
            // lastSeenStrafe = strafePidOutput;
            // }

            // if (rotationPidOutput != 0) {
            // lastSeenRotation = rotationPidOutput;
            // }
            System.out.println("I SEE THE TAG");

            ledCmd.schedule();
            ledOffCmd.cancel();

            mathToTag();
        } else {
            // swerveDrive.visionRotationVal(lastSeenRotation, true);
            // swerveDrive.visionStrafeVal(lastSeenStrafe, true);

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

        // Rotation
        double yaw = botPose[4] - cameraAngle;

        // Left/Right
        double zDiff = botPose3D.getZ() + desiredz;

        // forward/back
        double xDiff = botPose3D.getX() - xOffset;

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        System.out.println("I SHOULD CALLING DRIVE");

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
    }

    // Calculates and outpits PID based on difference to goal state (Forward
    // distance)
    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > Constants.Vision.distanceTolerance) {
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = translationPidOutput * Constants.Vision.distancePIDMultiplier; // PID multiplier
            if (translationPidOutput > Constants.Vision.distancePIDCap)
                translationPidOutput = Constants.Vision.distancePIDCap;
            limelight.setZPos(false);
        } else {
            translationPidOutput = 0;
            limelight.setZPos(true);
        }
    }

    // Calculates and outpits PID based on difference to goal state (Left/Right)
    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > Constants.Vision.strafeTolerance) {
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * Constants.Vision.strafePIDMultiplier; // PID multiplier
            if (strafePidOutput > Constants.Vision.strafePIDCap)
                strafePidOutput = Constants.Vision.strafePIDCap;
            if (strafePidOutput < -Constants.Vision.strafePIDCap)
                strafePidOutput = -Constants.Vision.strafePIDCap;

            limelight.setXPos(false);
        } else {
            strafePidOutput = 0;
            limelight.setXPos(true);
        }
    }

    // PID/Speed cap increase smaller adjustments within the PID but caps the max
    // allowing for slower/controlled overall speed but larger smaller adjustments

    public void setDrive() {
        swerveDrive.setFieldRelative(false);

        System.out.println("I SHOULD BE DRIVING RIGHT NOW");

        swerveDrive.visionRotationVal(rotationPidOutput, true);
        swerveDrive.visionTranslationalVal(translationPidOutput, true);
        swerveDrive.visionStrafeVal(strafePidOutput, true);

        swerveDrive.drive(true);
    }

    // only finishes when command canceled or when fully in position
    @Override
    public boolean isFinished() {
        return limelight.inPosition(); // Stop when aligned
    }

    // unlocks controls for driver and sets back to field releative
    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);

        tag = 0;
        tag2 = 0;

        swerveDrive.setFieldRelative(true);

        swerveDrive.resetSwerve();

        ledCmd.cancel();
        ledOffCmd.cancel();
    }
}