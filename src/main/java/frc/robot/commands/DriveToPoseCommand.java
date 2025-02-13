package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 *
 */
public class DriveToPoseCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final Supplier<Pose2d> poseProvider;
    private Pose2d goalPose;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

    /**
     * 
     */
    public DriveToPoseCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> poseProvider,
            Pose2d goalPose) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.poseProvider = poseProvider;
        this.goalPose = goalPose;

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

        this.swerveDriveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

    @Override
    public void initialize() {
        resetPIDControllers();

        // Flip pose if red alliance
        this.goalPose = AllianceFlipUtil.apply(goalPose);

        this.xController.setGoal(this.goalPose.getX());
        this.yController.setGoal(this.goalPose.getY());
        this.omegaController.setGoal(this.goalPose.getRotation().getRadians());

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
