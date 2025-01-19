package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToBestTagCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final String cameraName;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    private Transform3d ROBOT_TO_TAG;

    /**
     * Drive to a set distance (tag offset) away from vision system best tag
     */
    public DriveToBestTagCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem,
            Supplier<Pose2d> poseProvider, String cameraName) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;
        this.cameraName = cameraName;

        double[] driveXPIDs = PIDConstants.getDriveXPIDs();
        double[] driveYPIDs = PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();

        this.xController = new ProfiledPIDController(driveXPIDs[0], driveXPIDs[1], driveXPIDs[2],
                TeleopConstants.X_CONSTRAINTS);
        this.yController = new ProfiledPIDController(driveYPIDs[0], driveYPIDs[1], driveYPIDs[2],
                TeleopConstants.Y_CONSTRAINTS);
        this.omegaController = new ProfiledPIDController(driveOmegaPIDs[0], driveOmegaPIDs[1], driveOmegaPIDs[2],
                TeleopConstants.OMEGA_CONSTRAINTS);

        this.xController.setTolerance(0.2);
        this.yController.setTolerance(0.2);
        this.omegaController.setTolerance(Units.degreesToRadians(3));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.swerveDriveSubsystem);
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDriveSubsystem.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        Pose2d robotPose = this.poseProvider.get();

        double xSpeed = this.xController.calculate(robotPose.getX());
        double ySpeed = this.yController.calculate(robotPose.getY());
        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());

        if (this.xController.atGoal())
            xSpeed = 0;
        if (this.yController.atGoal())
            ySpeed = 0;
        if (this.omegaController.atGoal())
            omegaSpeed = 0;

        this.swerveDriveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

    @Override
    public void initialize() {
        resetPIDControllers();

        Pose2d robotPose2d = this.poseProvider.get();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        Pose2d goalPose = getBestTagPose(robotPose);
        if (goalPose == null) {
            goalPose = robotPose2d;
        }

        this.xController.setGoal(goalPose.getX());
        this.yController.setGoal(goalPose.getY());
        this.omegaController.setGoal(goalPose.getRotation().getRadians());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return isAtGoal();
    }

    /**
     * 
     */
    private Pose2d getBestTagPose(Pose3d currentPose) {
        // PhotonTrackedTarget target =
        // this.visionSubsystem.getBestTarget(this.cameraName);

        // if (target == null)
        // return null;

        // double cameraYaw =
        // this.visionSubsystem.getVisionModuleByName(this.cameraName).getCameraYaw();
        // double xMetersFromTag =
        // this.visionSubsystem.getTagOffset(target.getFiducialId());

        // if (xMetersFromTag < 0)
        // return null;

        // this.ROBOT_TO_TAG = new Transform3d(new Translation3d(xMetersFromTag, 0.0,
        // 0.0),
        // new Rotation3d(0.0, 0.0, cameraYaw));

        // Pose3d cameraPose =
        // currentPose.transformBy(this.visionSubsystem.getRobotToCamera(this.cameraName));
        // Transform3d camToTarget = target.getBestCameraToTarget();
        // Pose3d targetPose = cameraPose.transformBy(camToTarget);

        // return targetPose.transformBy(ROBOT_TO_TAG).toPose2d();
        return null;
    }

    /**
     * 
     */
    private boolean isAtGoal() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = this.poseProvider.get();

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }
}
