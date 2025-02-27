package frc.robot.commands.Vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * 
 */
public class DriveFromBestTagCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final boolean isLeftPole;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    /**
     * Drive to a provided translation/rotation away from vision system best tag
     */
    public DriveFromBestTagCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem visionSubsystem,
            Supplier<Pose2d> poseProvider, boolean isLeftPole) {
        this.swerveDrive = swerveDrive;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;
        this.isLeftPole = isLeftPole;

        double[] driveXPIDs = PIDConstants.getDriveXPIDs();
        double[] driveYPIDs = PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();

        this.xController = new ProfiledPIDController(driveXPIDs[0], driveXPIDs[1], driveXPIDs[2],
                TeleopConstants.X_CONSTRAINTS);
        this.yController = new ProfiledPIDController(driveYPIDs[0], driveYPIDs[1], driveYPIDs[2],
                TeleopConstants.Y_CONSTRAINTS);
        this.omegaController = new ProfiledPIDController(driveOmegaPIDs[0], driveOmegaPIDs[1], driveOmegaPIDs[2],
                TeleopConstants.OMEGA_CONSTRAINTS);

        this.xController.setTolerance(0.01);
        this.yController.setTolerance(0.01);
        this.omegaController.setTolerance(Units.degreesToRadians(1));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.swerveDrive);
        addRequirements(this.visionSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        Pose2d robotPose = this.poseProvider.get();

        double xSpeed = this.xController.calculate(robotPose.getX());
        double ySpeed = this.yController.calculate(robotPose.getY());
        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());

        System.out.printf("*** SPEEDS - X: %.2f - Y: %.2f - O: %.2f", xSpeed, ySpeed, omegaSpeed);
        if (this.xController.atGoal())
            xSpeed = 0;
        if (this.yController.atGoal())
            ySpeed = 0;
        if (this.omegaController.atGoal())
            omegaSpeed = 0;

        this.swerveDrive.drive(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation(), false);
    }

    /**
     * 
     */
    private Pose2d getBestTagPose(Pose3d currentPose) {
        boolean isAlgaeMode = false; // hard code for coral subsystem this.gamePieceModeSupplier.get() ==
                                     // GamePieceMode.ALGAE;

        String cameraName = isAlgaeMode
                ? VisionConstants.REAR_CAMERA.getName()
                : VisionConstants.FRONT_CAMERA.getName();
        Pose3d targetPose = this.visionSubsystem.getBestTargetPose(cameraName);

        // return null if we don't have a best tag
        if (targetPose == null || targetPose.equals(new Pose3d())) {
            return null;
        }

        double yOffset = 0.0;
        double yawOffset = 0.0;
        if (!isAlgaeMode) {
            yOffset = isLeftPole ? -FieldConstants.REEF_POLE_LEFT_OFFSET : FieldConstants.REEF_POLE_RIGHT_OFFSET;
            yawOffset = Units.degreesToRadians(180.0);
        }
        Transform3d transformation = new Transform3d(
                new Translation3d(RobotConstants.LENGTH_METERS / 2, yOffset, 0.0),
                new Rotation3d(0.0, 0.0, yawOffset));

        return targetPose.transformBy(transformation).toPose2d();
    }

    @Override
    public void initialize() {
        resetPIDControllers();

        Pose2d robotPose2d = this.poseProvider.get();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        Pose2d goalPose = getBestTagPose(robotPose);
        if (goalPose == null)
            goalPose = robotPose2d;

        this.xController.setGoal(goalPose.getX());
        this.yController.setGoal(goalPose.getY());
        this.omegaController.setGoal(goalPose.getRotation().getRadians());

        Logger.recordOutput("Commands/Goal Pose", goalPose);
        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    /**
     * 
     */
    private boolean isAtGoal() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
    }

    @Override
    public boolean isFinished() {
        return isAtGoal();
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
