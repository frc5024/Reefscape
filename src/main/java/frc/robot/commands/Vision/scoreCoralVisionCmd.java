package frc.robot.commands.Vision;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.Swerve;

public class scoreCoralVisionCmd extends Command {
    static ShuffleboardTab tab = Shuffleboard.getTab("Tags");
    // GenericEntry pEntry = tab.add("SET P VISION", 0.7).getEntry();
    // GenericEntry iEntry = tab.add("SET I VISION", 0).getEntry();
    // GenericEntry dEntry = tab.add("SET D VISION", 0.05).getEntry();

    private final Limelight limelight;
    private final Swerve swerveDrive;
    private final Elevator elevatorSubsystem;
    private final Coral coralSubsystem;
    private Rumble rumbleSubsystem;

    private final double xOffset;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    double desiredz = 0.430; // in meters
    double desiredx = 0; // in meters? (+ right)?
    double elevatorDistance = 1.5;

    double tagAngle = 0;
    private final double cameraAngle = 29;

    boolean xPos = false;
    boolean rotationPos = false;
    boolean zPos = false;

    boolean finished = false;
    boolean elevatorSet = false;
    boolean scored = false;

    public scoreCoralVisionCmd(Limelight limelight, Swerve swerveDrive, double xOffset, Elevator elevatorSubsystem,
            Coral coralSubsystem) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.xOffset = xOffset;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;

        this.strafePidController = new PIDController(0.7, 0, 0.05);
        this.translationPidController = new PIDController(0.5, 0, 0.05);
        this.rotationPidController = new PIDController(0.008, 0, 0.0005);

        // addRequirements(elevatorSubsystem, coralSubsystem);
    }

    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();

        xPos = false;
        zPos = false;
        rotationPos = false;

        finished = false;
        elevatorSet = false;
        scored = false;

        elevatorSubsystem.resetPID();
    }

    Set<Integer> validTagIDs = new HashSet<>(
            Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
    // 6, 7, 8, 9, 10, 11 - red reef
    // 17, 18, 19, 20, 21, 22 - blue reef

    @Override
    public void execute() {
        int detectedTagID = (int) limelight.getAprilTagID();

        if (validTagIDs.contains(detectedTagID)) {
            swerveDrive.setFieldRelative(false);

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

        SmartDashboard.putNumber("Tag Yaw V", yaw);

        // Left/Right
        double zDiff = botPose3D.getZ() + desiredz;

        // forward/back
        double xDiff = botPose3D.getX() - xOffset;

        rotateToTag(yaw);
        translateToTag(zDiff);
        strafeToTag(xDiff);

        setDrive();

        // activeFunctions();
    }

    public void rotateToTag(double rotationToTag) {
        if (Math.abs(rotationToTag) > 1.5) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(rotationToTag, 0);
            rotationPidOutput = rotationPidOutput * 2.2; // Speed multiplier
            rotationPos = false;
        } else {
            rotationPidOutput = 0;
            rotationPos = true;
        }
    }

    public void translateToTag(double zDiff) {
        if (Math.abs(zDiff) > 0.06) { // In meters
            translationPidOutput = translationPidController.calculate(zDiff, 0);
            translationPidOutput = translationPidOutput * 2.6; // Speed multiplier (1.2)
            if (translationPidOutput > 0.3)
                translationPidOutput = 0.3;
            zPos = false;
        } else {
            translationPidOutput = 0;
            zPos = true;
        }
    }

    public void strafeToTag(double xDiff) {
        if (Math.abs(xDiff) > 0.025) { // In meters
            strafePidOutput = strafePidController.calculate(xDiff, 0);
            strafePidOutput = -strafePidOutput * 2.3; // Speed multiplier
            if (strafePidOutput > 0.2)
                strafePidOutput = 0.2;
            if (strafePidOutput < -0.2)
                strafePidOutput = -0.2;

            xPos = false;
        } else {
            strafePidOutput = 0;
            xPos = true;
        }
    }

    public void setDrive() {
        swerveDrive.setFieldRelative(false);

        if (xPos && zPos && rotationPos)
            rumbleSubsystem.staticRumble(true);

        SmartDashboard.putBoolean("rotationPos V", rotationPos);
        SmartDashboard.putBoolean("xPos V", xPos);
        SmartDashboard.putBoolean("zPos V", zPos);

        swerveDrive.visionRotationVal(rotationPidOutput, true);
        swerveDrive.visionTranslationalVal(translationPidOutput, true);
        swerveDrive.visionStrafeVal(strafePidOutput, true);
    }

    public void elevator() {
        if (!elevatorSet) {
            elevatorSet = true;
            elevatorSubsystem.setGoal(Constants.elevatorConstants.L4Position);
            elevatorSubsystem.togglePID(true);
            if (elevatorSubsystem.getElevatorMode() != Constants.elevatorConstants.rootPosition) {
                elevatorSubsystem.setElevatorPosition(elevatorSubsystem.getElevatorMode());
            }
        }
    }

    public void activeFunctions() {
        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");

        if (botPose3D.getX() < elevatorDistance) {
            elevator();
        }

        if (elevatorSubsystem.targetReached() && xPos && zPos && rotationPos) {
            coralSubsystem.outtakeCommand();
            scored = true;
        }
    }

    @Override
    public boolean isFinished() {
        return scored; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);

        if (scored)
            rumbleSubsystem.staticRumble(true);

        elevatorSubsystem.bottomElevator();
        coralSubsystem.cancelIntakeCommand();

        swerveDrive.setFieldRelative(true);
    }
}