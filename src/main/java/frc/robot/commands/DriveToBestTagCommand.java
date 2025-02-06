package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.controls.GameData.CoralPole;
import frc.robot.controls.GameData.GamePieceMode;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.AllianceFlipUtil;

public class DriveToBestTagCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final Supplier<CoralPole> poleSupplier;
    private final Supplier<GamePieceMode> gamePieceModeSupplier;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    /**
     * Drive to best tag plus any pole offset
     */
    public DriveToBestTagCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem,
            Supplier<Pose2d> poseProvider, Supplier<CoralPole> poleSupplier,
            Supplier<GamePieceMode> gamePieceModeSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;
        this.poleSupplier = poleSupplier;
        this.gamePieceModeSupplier = gamePieceModeSupplier;

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

        // Flip pose if red alliance
        goalPose = AllianceFlipUtil.apply(goalPose);

        // Determine if we want to drive to coral pole or algae
        GamePieceMode driveMode = this.gamePieceModeSupplier.get();
        double robotYaw = 0.0;
        if (driveMode == GamePieceMode.CORAL) {
            robotYaw = 180.0;
        }

        this.xController.setGoal(goalPose.getX());
        this.yController.setGoal(goalPose.getY());
        this.omegaController.setGoal(goalPose.getRotation().rotateBy(Rotation2d.fromDegrees(robotYaw)).getRadians());

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
        String cameraName = this.gamePieceModeSupplier.get() == GamePieceMode.ALGAE
                ? VisionConstants.REAR_CAMERA.getName()
                : VisionConstants.FRONT_CAMERA.getName();
        Pose3d targetPose = this.visionSubsystem.getBestTargetPose(cameraName);

        if (targetPose == null) {
            return null;
        }

        CoralPole poleId = this.poleSupplier.get();
        GamePieceMode gamePieceMode = this.gamePieceModeSupplier.get();

        // Set pole offset if Coral
        double offset = 0.0;
        if (gamePieceMode == GamePieceMode.CORAL) {
            offset = poleId == CoralPole.LEFT ? FieldConstants.REEF_POLE_OFFSET : -FieldConstants.REEF_POLE_OFFSET;
        }

        Transform3d ROBOT_TO_TAG = new Transform3d(new Translation3d(RobotConstants.LENGTH_METERS / 2, offset, 0.0),
                new Rotation3d(0.0, 0.0, 0.0));

        return targetPose.transformBy(ROBOT_TO_TAG).toPose2d();
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
