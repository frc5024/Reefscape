package frc.robot.commands.vision;

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
import frc.robot.controls.GameData.CoralPole;
import frc.robot.controls.GameData.GamePieceMode;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * Drives to reef station based on game piece mode settings
 */
public class DriveReefStationCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private final Supplier<Integer> stationSupplier;
    private final Supplier<CoralPole> poleSupplier;
    private final Supplier<GamePieceMode> driveModeSupplier;
    private Pose3d goalPose;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    /**
     * Drives to reef station based on pose and pole selection from elastic input
     */
    public DriveReefStationCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> poseProvider,
            Supplier<Integer> stationSupplier, Supplier<CoralPole> poleSupplier,
            Supplier<GamePieceMode> driveModeSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.poseProvider = poseProvider;
        this.stationSupplier = stationSupplier;
        this.poleSupplier = poleSupplier;
        this.driveModeSupplier = driveModeSupplier;

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

        addRequirements(this.swerveDriveSubsystem);
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

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
                robotPose.getRotation());
        this.swerveDriveSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void initialize() {
        resetPIDControllers();

        int stationId = this.stationSupplier.get().intValue();
        CoralPole poleId = this.poleSupplier.get();
        GamePieceMode driveMode = this.driveModeSupplier.get();

        // Determine if we want to drive to left or right coral pole
        Pose3d reefPose3d = new Pose3d(FieldConstants.REEF_POSES[stationId - 1]);
        Pose2d reefPose2d = AllianceFlipUtil.apply(reefPose3d.toPose2d());
        this.goalPose = new Pose3d(reefPose2d);

        double offset = poleId == CoralPole.LEFT ? FieldConstants.REEF_POLE_OFFSET : -FieldConstants.REEF_POLE_OFFSET;
        double robotYaw = driveMode == GamePieceMode.CORAL ? 0.0 : 180.0;

        Transform3d polePose = new Transform3d(new Translation3d(RobotConstants.LENGTH_METERS / 2, offset, 0.0),
                new Rotation3d(0.0, 0.0, 0.0));
        Pose2d driveToPose = this.goalPose.transformBy(polePose).toPose2d();

        this.xController.setGoal(reefPose2d.getX());
        this.yController.setGoal(reefPose2d.getY());
        this.omegaController.setGoal(reefPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(robotYaw)).getRadians());

        Logger.recordOutput("Commands/Goal Pose", goalPose);
        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = poseProvider.get();

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }
}
